function tests = test_matlab_component_builder
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
toolRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(toolRoot, 'matlab'));
paths = gmp_mcb.build_all();
libraryPath = gmp_mcb.create_library();
addpath(paths.installDir);
addpath(paths.mexDir);
load_system(libraryPath);
testCase.TestData.toolRoot = toolRoot;
testCase.TestData.paths = paths;
end

function teardownOnce(~)
if bdIsLoaded('gmp_mcb_components'), close_system('gmp_mcb_components', 0); end
end

function testGeneratedLibraryAndMask(testCase)
blocks = find_system('gmp_mcb_components', 'BlockType', 'S-Function');
verifyEqual(testCase, numel(blocks), 1);
verifyEqual(testCase, get_param(blocks{1}, 'FunctionName'), 'gmp_mcb_intrinsic_continuous_pid');
verifyEqual(testCase, get_param(blocks{1}, 'MaskType'), 'GMP MATLAB Component Builder PID');
mask = Simulink.Mask.get(blocks{1});
verifyGreaterThanOrEqual(testCase, numel(mask.Parameters), 10);
end

function testParallelFirstSamplesHaveNoExtraDelay(testCase)
model = 'gmp_mcb_unit_sample';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block('simulink/Sources/From Workspace', [model '/In'], 'VariableName', 'u');
add_block('gmp_mcb_components/GMPContinuous_FormDiscretePID', [model '/PID']);
add_block('simulink/Sinks/To Workspace', [model '/Out'], ...
    'VariableName', 'y', 'SaveFormat', 'Array');
add_line(model, 'In/1', 'PID/1');
add_line(model, 'PID/1', 'Out/1');
set_param([model '/PID'], ...
    'init_method', 'Parallel gains (Kp, Ki, Kd)', ...
    'kp', '0.4', 'ki_or_ti', '20', 'kd_or_td', '0.0001', ...
    'fs', '10000', 'analysis_fs', '10000', ...
    'out_max', '1e6', 'out_min', '-1e6', ...
    'integral_max', '1e6', 'integral_min', '-1e6');
ts = 1e-4;
u = timeseries(0.1 * ones(5, 1), (0:4)' * ts);
simulationInput = Simulink.SimulationInput(model).setVariable('u', u).setModelParameter( ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', '1e-4', 'StopTime', '4e-4');
simulationOutput = sim(simulationInput);
verifyEqual(testCase, simulationOutput.y(1:3), [0.1402; 0.0404; 0.0406], 'AbsTol', 2e-6);
clear cleanup;
close_model(model);
end

function testIndependentExecutionFrequencyModel(testCase)
model = 'gmp_mcb_unit_rate';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block('gmp_mcb_components/GMPContinuous_FormDiscretePID', [model '/PID']);
set_param([model '/PID'], 'fs', '10000', 'analysis_fs', '5000');
models = gmp_mcb.pid_frequency_models([model '/PID'], [100; 500]);
verifyEqual(testCase, models.parameterFs, 10000);
verifyEqual(testCase, models.executionFs, 5000);
verifySize(testCase, models.implementation, [2 1]);
clear cleanup;
close_model(model);
end

function close_model(model)
if bdIsLoaded(model), close_system(model, 0); end
end

