function result = measure_component_block(block, frequenciesHz)
% Measure a compiled generated S-Function with a coherent sequential sine sweep.
maskMetadata = Simulink.Mask.get(block);
if isempty(maskMetadata.getParameter('fs'))
    parameterFs = NaN;
else
    parameterFs = gmp_mcb.mask_value(block, 'fs');
end
executionFs = gmp_mcb.mask_value(block, 'analysis_execution_fs');
if nargin < 2 || isempty(frequenciesHz)
    fmin = gmp_mcb.mask_value(block, 'analysis_frequency_min');
    fmax = gmp_mcb.mask_value(block, 'analysis_frequency_max');
    points = round(gmp_mcb.mask_value(block, 'analysis_points'));
    frequenciesHz = logspace(log10(fmin), log10(fmax), points);
end
frequenciesHz = frequenciesHz(:);
amplitude = gmp_mcb.mask_value(block, 'analysis_amplitude');
settlingPeriods = round(gmp_mcb.mask_value(block, 'analysis_settling_periods'));
measurementPeriods = round(gmp_mcb.mask_value(block, 'analysis_measurement_periods'));
if executionFs <= 0 || amplitude <= 0 || settlingPeriods < 0 || measurementPeriods < 1
    error('GMP:MCB:AnalysisSettings', 'Invalid standalone analysis settings.');
end
ts = 1 / executionFs;

model = ['gmp_mcb_measure_' char(java.util.UUID.randomUUID().toString().replace('-', '_'))];
new_system(model);
cleanup = onCleanup(@() cleanup_model(model));
add_block('simulink/Sources/From Workspace', [model '/Excitation'], ...
    'VariableName', 'mcb_input', 'Interpolate', 'on', 'Position', [40 70 150 100]);
dut = [model '/DUT'];
add_block(block, dut, 'Position', [220 55 430 115]);
mask = Simulink.Mask.get(dut);
for index = 1:numel(mask.Parameters)
    name = mask.Parameters(index).Name;
    if startsWith(name, 'expose_'), set_param(dut, name, 'off'); end
end
add_block('simulink/Sinks/To Workspace', [model '/Response'], ...
    'VariableName', 'mcb_output', 'SaveFormat', 'Structure With Time', ...
    'Position', [500 70 610 100]);
add_line(model, 'Excitation/1', 'DUT/1');
add_line(model, 'DUT/1', 'Response/1');
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(ts, 17), ...
    'ReturnWorkspaceOutputs', 'on');

response = complex(zeros(size(frequenciesHz)));
actualFrequency = zeros(size(frequenciesHz));
for index = 1:numel(frequenciesHz)
    samplesPerPeriod = max(3, round(executionFs / frequenciesHz(index)));
    frequency = executionFs / samplesPerPeriod;
    sampleCount = (settlingPeriods + measurementPeriods) * samplesPerPeriod;
    time = (0:sampleCount)' * ts;
    input = amplitude * sin(2 * pi * frequency * time);
    simulationInput = Simulink.SimulationInput(model);
    simulationInput = simulationInput.setVariable('mcb_input', timeseries(input, time));
    simulationInput = simulationInput.setModelParameter('StopTime', num2str(time(end), 17));
    simulationOutput = sim(simulationInput);
    logged = simulationOutput.get('mcb_output');
    output = logged.signals.values(:);
    count = min(numel(output), numel(input));
    keep = measurementPeriods * samplesPerPeriod;
    indices = (count - keep + 1):count;
    kernel = exp(-1i * 2 * pi * frequency * time(indices));
    response(index) = sum(output(indices) .* kernel) / sum(input(indices) .* kernel);
    actualFrequency(index) = frequency;
end

models = gmp_mcb.component_frequency_models(block, actualFrequency);
result = struct('frequencyHz', actualFrequency, 'measured', response, ...
    'continuous', models.continuous, 'implementation', models.implementation, ...
    'amplitude', amplitude, 'parameterFs', parameterFs, 'executionFs', executionFs);
cacheDir = fullfile(gmp_mcb.tool_root(), 'cache');
if ~isfolder(cacheDir), mkdir(cacheDir); end
cacheFile = fullfile(cacheDir, ['component_' datestr(now, 'yyyymmdd_HHMMSS_FFF') '.mat']);
save(cacheFile, 'result');

figure('Name', ['Measured GMP Component: ' get_param(block, 'Name')]);
tiledlayout(2, 1);
nexttile;
semilogx(actualFrequency, 20 * log10(abs(models.continuous)), 'LineWidth', 1.2); hold on;
semilogx(actualFrequency, 20 * log10(abs(models.implementation)), '--', 'LineWidth', 1.2);
semilogx(actualFrequency, 20 * log10(abs(response)), 'o', 'LineWidth', 1.2);
grid on; ylabel('Magnitude [dB]');
legend('Ideal continuous', 'Implementation reference', 'Measured MEX', 'Location', 'best');
nexttile;
semilogx(actualFrequency, unwrap(angle(models.continuous)) * 180 / pi, 'LineWidth', 1.2); hold on;
semilogx(actualFrequency, unwrap(angle(models.implementation)) * 180 / pi, '--', 'LineWidth', 1.2);
semilogx(actualFrequency, unwrap(angle(response)) * 180 / pi, 'o', 'LineWidth', 1.2);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
fprintf('Measurement cache: %s\n', cacheFile);
end

function cleanup_model(model)
if bdIsLoaded(model), close_system(model, 0); end
end
