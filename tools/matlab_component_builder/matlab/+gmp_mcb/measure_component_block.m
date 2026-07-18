function result = measure_component_block(block, frequenciesHz)
% Measure a compiled generated S-Function with a coherent sequential sine sweep.
maskMetadata = Simulink.Mask.get(block);
component = get_param(block, 'UserData');
if ~isstruct(component) || ~isfield(component, 'inputs') || ~isfield(component, 'outputs')
    error('GMP:MCB:Metadata', 'Block does not contain generated component port metadata.');
end
inputIndex = selected_port_index(block, maskMetadata, component.inputs, 'analysis_input_port');
outputIndex = selected_port_index(block, maskMetadata, component.outputs, 'analysis_output_port');
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
bias = gmp_mcb.mask_value(block, 'analysis_bias');
settlingPeriods = round(gmp_mcb.mask_value(block, 'analysis_settling_periods'));
measurementPeriods = round(gmp_mcb.mask_value(block, 'analysis_measurement_periods'));
if executionFs <= 0 || amplitude <= 0 || settlingPeriods < 0 || measurementPeriods < 1 || ...
        ~isfinite(bias) || isempty(frequenciesHz) || any(~isfinite(frequenciesHz)) || any(frequenciesHz <= 0) || ...
        any(frequenciesHz >= executionFs / 2)
    error('GMP:MCB:AnalysisSettings', 'Invalid standalone analysis settings.');
end
ts = 1 / executionFs;

model = ['gmp_mcb_measure_' char(java.util.UUID.randomUUID().toString().replace('-', '_'))];
new_system(model);
cleanup = onCleanup(@() cleanup_model(model));
dut = [model '/DUT'];
add_block(block, dut, 'Position', [220 55 430 115]);
mask = Simulink.Mask.get(dut);
for index = 1:numel(mask.Parameters)
    name = mask.Parameters(index).Name;
    if startsWith(name, 'expose_'), set_param(dut, name, 'off'); end
end
set_param(model, 'SimulationCommand', 'update');

operatingPoints = zeros(numel(component.inputs), 1);
for port = 1:numel(component.inputs)
    input = component.inputs(port);
    if port == inputIndex
        operatingPoints(port) = bias;
        source = [model '/Excitation'];
        add_block('simulink/Sources/From Workspace', source, ...
            'VariableName', 'mcb_input', 'Interpolate', 'on', ...
            'Position', [40 40 + 55 * port 150 70 + 55 * port]);
    else
        parameterName = ['analysis_operating_' char(input.id)];
        operatingPoints(port) = optional_mask_value(block, parameterName, 0);
        source = sprintf('%s/OperatingPoint%d', model, port);
        add_block('simulink/Sources/Constant', source, ...
            'Value', num2str(operatingPoints(port), 17), ...
            'Position', [60 40 + 55 * port 130 70 + 55 * port]);
    end
    add_line(model, [get_param(source, 'Name') '/1'], sprintf('DUT/%d', port));
end

add_block('simulink/Sinks/To Workspace', [model '/Response'], ...
    'VariableName', 'mcb_output', 'SaveFormat', 'Structure With Time', ...
    'Position', [500 70 610 100]);
for port = 1:numel(component.outputs)
    if port == outputIndex
        add_line(model, sprintf('DUT/%d', port), 'Response/1');
    else
        terminator = sprintf('%s/UnusedOutput%d', model, port);
        add_block('simulink/Sinks/Terminator', terminator, ...
            'Position', [520 40 + 55 * port 540 60 + 55 * port]);
        add_line(model, sprintf('DUT/%d', port), [get_param(terminator, 'Name') '/1']);
    end
end
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(ts, 17), ...
    'ReturnWorkspaceOutputs', 'on');

response = complex(zeros(size(frequenciesHz)));
actualFrequency = zeros(size(frequenciesHz));
settlingReferenceHz = NaN;
if isfield(component.analysis, 'settling_reference_input')
    referenceId = string(component.analysis.settling_reference_input);
    referenceIndex = find(string({component.inputs.id}) == referenceId, 1);
    if isempty(referenceIndex) || referenceIndex == inputIndex
        error('GMP:MCB:AnalysisSettings', ...
            'Settling reference input must identify a non-excited input port.');
    end
    settlingReferenceHz = operatingPoints(referenceIndex);
    if ~isfinite(settlingReferenceHz) || settlingReferenceHz <= 0 || settlingReferenceHz >= executionFs / 2
        error('GMP:MCB:AnalysisSettings', 'Locked frequency must lie between 0 and Nyquist.');
    end
end
for index = 1:numel(frequenciesHz)
    samplesPerPeriod = max(3, round(executionFs / frequenciesHz(index)));
    frequency = executionFs / samplesPerPeriod;
    settlingSamples = settlingPeriods * samplesPerPeriod;
    if isfinite(settlingReferenceHz)
        lockedSamplesPerPeriod = max(3, round(executionFs / settlingReferenceHz));
        settlingSamples = settlingPeriods * lockedSamplesPerPeriod;
    end
    sampleCount = settlingSamples + measurementPeriods * samplesPerPeriod;
    time = (0:sampleCount)' * ts;
    excitation = amplitude * sin(2 * pi * frequency * time);
    input = bias + excitation;
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
    response(index) = sum(output(indices) .* kernel) / sum(excitation(indices) .* kernel);
    actualFrequency(index) = frequency;
end

models = gmp_mcb.component_frequency_models(block, actualFrequency);
result = struct('frequencyHz', actualFrequency, 'measured', response, ...
    'continuous', models.continuous, 'implementation', models.implementation, ...
    'hasReferenceModel', models.hasReferenceModel, ...
    'amplitude', amplitude, 'bias', bias, 'parameterFs', parameterFs, 'executionFs', executionFs, ...
    'settlingPeriods', settlingPeriods, 'measurementPeriods', measurementPeriods, ...
    'settlingReferenceHz', settlingReferenceHz, ...
    'componentId', string(component.id), 'componentName', string(component.display_name), ...
    'inputIndex', inputIndex, 'inputLabel', string(component.inputs(inputIndex).label), ...
    'outputIndex', outputIndex, 'outputLabel', string(component.outputs(outputIndex).label), ...
    'operatingPoints', operatingPoints);
cacheDir = fullfile(gmp_mcb.tool_root(), 'cache');
if ~isfolder(cacheDir), mkdir(cacheDir); end
cacheFile = fullfile(cacheDir, ['component_' datestr(now, 'yyyymmdd_HHMMSS_FFF') '.mat']);
save(cacheFile, 'result');

gmp_mcb.plot_measurement_result(result, get_param(block, 'Name'));
fprintf('Measurement cache: %s\n', cacheFile);
end

function cleanup_model(model)
if bdIsLoaded(model), close_system(model, 0); end
end

function index = selected_port_index(block, mask, ports, parameterName)
parameter = mask.getParameter(parameterName);
if isempty(parameter)
    index = 1;
    return;
end
selected = string(get_param(block, parameterName));
labels = string({ports.label});
index = find(labels == selected, 1);
if isempty(index)
    error('GMP:MCB:AnalysisPort', 'Selected analysis port %s is invalid.', selected);
end
end

function value = optional_mask_value(block, name, defaultValue)
mask = Simulink.Mask.get(block);
if isempty(mask.getParameter(name))
    value = defaultValue;
else
    value = gmp_mcb.mask_value(block, name);
end
end
