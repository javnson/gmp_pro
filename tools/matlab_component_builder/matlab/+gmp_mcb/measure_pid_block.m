function result = measure_pid_block(block, frequenciesHz)
% Measure the compiled S-Function with a coherent sequential sine sweep.
parameterFs = gmp_mcb.mask_value(block, 'fs');
executionFs = gmp_mcb.mask_value(block, 'analysis_fs');
if nargin < 2 || isempty(frequenciesHz)
    frequenciesHz = logspace(log10(max(executionFs / 5000, 1)), log10(min(0.4 * executionFs, 2000)), 16);
end
frequenciesHz = frequenciesHz(:);
amplitude = min(1e-3, 0.01 * max(1, abs(gmp_mcb.mask_value(block, 'out_max'))));
settlingPeriods = 4;
measurementPeriods = 6;
ts = 1 / executionFs;

model = ['gmp_mcb_measure_' char(java.util.UUID.randomUUID().toString().replace('-', '_'))];
new_system(model);
cleanup = onCleanup(@() cleanup_model(model));
add_block('simulink/Sources/From Workspace', [model '/Excitation'], ...
    'VariableName', 'mcb_input', 'Interpolate', 'on', 'Position', [40 70 150 100]);
add_block(block, [model '/DUT'], 'Position', [220 55 430 115]);
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
    inputCoefficient = sum(input(indices) .* kernel);
    outputCoefficient = sum(output(indices) .* kernel);
    response(index) = outputCoefficient / inputCoefficient;
    actualFrequency(index) = frequency;
end

models = gmp_mcb.pid_frequency_models(block, actualFrequency);
result = struct('frequencyHz', actualFrequency, 'measured', response, ...
    'continuous', models.continuous, 'implementation', models.implementation, ...
    'amplitude', amplitude, 'parameterFs', parameterFs, 'executionFs', executionFs);

cacheDir = fullfile(gmp_mcb.tool_root(), 'cache');
if ~isfolder(cacheDir), mkdir(cacheDir); end
cacheFile = fullfile(cacheDir, ['pid_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']);
save(cacheFile, 'result');

figure('Name', ['Measured GMP PID: ' get_param(block, 'Name')]);
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
