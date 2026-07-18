function tests = test_matlab_component_builder
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
toolRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(toolRoot, 'matlab'));
verifyMexDir = fullfile(toolRoot, 'build', 'test_mex');
paths = gmp_mcb.build_all(verifyMexDir);
libraryPath = gmp_mcb.create_library();
addpath(paths.mexDir, '-begin');
load_system(libraryPath);
testCase.TestData.toolRoot = toolRoot;
end

function teardownOnce(~)
if bdIsLoaded('gmp_mcb_components'), close_system('gmp_mcb_components', 0); end
end

function testGeneratedLibraryAndTabbedMasks(testCase)
blocks = find_system('gmp_mcb_components', 'BlockType', 'S-Function');
registry = gmp_mcb.load_registry();
verifyEqual(testCase, numel(blocks), numel(registry.components));
functions = string(get_param(blocks, 'FunctionName'));
verifyTrue(testCase, any(functions == "gmp_mcb_intrinsic_continuous_pid"));
verifyTrue(testCase, any(functions == "gmp_mcb_intrinsic_discrete_resonant_qpr"));
for index = 1:numel(blocks)
    verifyEqual(testCase, get_param(blocks{index}, 'MaskType'), 'GMP MATLAB Component Builder SISO');
    mask = Simulink.Mask.get(blocks{index});
    tabs = mask.getDialogControl('mcb_tabs');
    verifyEqual(testCase, string({tabs.DialogControls.Name}), ["parameters_tab" "analysis_tab"]);
end
end

function testParallelFirstSamplesHaveNoExtraDelay(testCase)
model = 'gmp_mcb_unit_sample';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block('simulink/Sources/From Workspace', [model '/In'], 'VariableName', 'u');
add_block(pid_library_block(), [model '/PID']);
add_block('simulink/Sinks/To Workspace', [model '/Out'], 'VariableName', 'y', 'SaveFormat', 'Array');
add_line(model, 'In/1', 'PID/1');
add_line(model, 'PID/1', 'Out/1');
set_param([model '/PID'], 'init_method', 'Parallel gains (Kp, Ki, Kd)', ...
    'kp', '0.4', 'ki_or_ti', '20', 'kd_or_td', '0.0001', 'fs', '10000', ...
    'analysis_execution_fs', '10000', 'out_max', '1e6', 'out_min', '-1e6', ...
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

function testExternalParameterPortsOverrideMaskValues(testCase)
model = 'gmp_mcb_unit_external';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block('simulink/Sources/Constant', [model '/Error'], 'Value', '0.1');
add_block('simulink/Sources/Constant', [model '/Kp'], 'Value', '2');
add_block(pid_library_block(), [model '/PID']);
set_param([model '/PID'], 'kp', '0.4', 'ki_or_ti', '0', 'kd_or_td', '0', ...
    'out_max', '1e6', 'out_min', '-1e6', 'integral_max', '1e6', 'integral_min', '-1e6', ...
    'expose_kp', 'on');
add_block('simulink/Sinks/To Workspace', [model '/Out'], 'VariableName', 'y', 'SaveFormat', 'Array');
add_line(model, 'Error/1', 'PID/1');
add_line(model, 'Kp/1', 'PID/2');
add_line(model, 'PID/1', 'Out/1');
simulationOutput = sim(Simulink.SimulationInput(model).setModelParameter( ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', '1e-4', 'StopTime', '1e-4'));
verifyEqual(testCase, simulationOutput.y(1), 0.2, 'AbsTol', 1e-8);
mask = Simulink.Mask.get([model '/PID']);
gmp_mcb.update_external_parameter([model '/PID'], 'kp');
verifyEqual(testCase, mask.getParameter('kp').Enabled, 'off');
clear cleanup;
close_model(model);
end

function testIndependentAnalysisFrequency(testCase)
model = 'gmp_mcb_unit_rate';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block(pid_library_block(), [model '/PID']);
set_param([model '/PID'], 'fs', '10000', 'analysis_execution_fs', '5000');
models = gmp_mcb.pid_frequency_models([model '/PID'], [100; 500]);
verifyEqual(testCase, models.parameterFs, 10000);
verifyEqual(testCase, models.executionFs, 5000);
verifySize(testCase, models.implementation, [2 1]);
clear cleanup;
close_model(model);
end

function testQprMeasuredResponseMatchesImplementation(testCase)
model = 'gmp_mcb_unit_qpr';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block(qpr_library_block(), [model '/QPR']);
set_param([model '/QPR'], 'analysis_execution_fs', '10000', ...
    'analysis_settling_periods', '20', 'analysis_measurement_periods', '8');
result = gmp_mcb.measure_component_block([model '/QPR'], [20; 50; 100]);
relativeError = abs(result.measured - result.implementation) ./ max(abs(result.implementation), eps);
verifyLessThan(testCase, relativeError, 0.03 * ones(size(relativeError)));
close all force;
clear cleanup;
close_model(model);
end

function testGenericSaturationAndExternalLimit(testCase)
model = 'gmp_mcb_unit_saturation';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block('simulink/Sources/Constant', [model '/Input'], 'Value', '2');
add_block('simulink/Sources/Constant', [model '/Maximum'], 'Value', '0.25');
add_block(library_block('gmp_mcb_intrinsic_basic_saturation'), [model '/Saturation']);
set_param([model '/Saturation'], 'expose_out_max', 'on');
add_block('simulink/Sinks/To Workspace', [model '/Out'], 'VariableName', 'y', 'SaveFormat', 'Array');
add_line(model, 'Input/1', 'Saturation/1');
add_line(model, 'Maximum/1', 'Saturation/2');
add_line(model, 'Saturation/1', 'Out/1');
simulationOutput = sim(Simulink.SimulationInput(model).setModelParameter( ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', '1e-4', 'StopTime', '1e-4'));
verifyEqual(testCase, simulationOutput.y(1), 0.25, 'AbsTol', 1e-8);
clear cleanup;
close_model(model);
end

function testMimoLadrcPortsCompile(testCase)
model = 'gmp_mcb_unit_ladrc';
cleanup = onCleanup(@() close_model(model));
new_system(model);
for index = 1:4
    add_block('simulink/Sources/Constant', sprintf('%s/In%d', model, index), 'Value', '0');
end
add_block(library_block('gmp_mcb_intrinsic_continuous_ladrc2'), [model '/LADRC2']);
set_param(model, 'SimulationCommand', 'update');
for index = 1:3
    add_block('simulink/Sinks/Terminator', sprintf('%s/Out%d', model, index));
end
for index = 1:4, add_line(model, sprintf('In%d/1', index), sprintf('LADRC2/%d', index)); end
for index = 1:3, add_line(model, sprintf('LADRC2/%d', index), sprintf('Out%d/1', index)); end
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', '1e-4', 'StopTime', '1e-4');
set_param(model, 'SimulationCommand', 'update');
ports = get_param([model '/LADRC2'], 'Ports');
verifyEqual(testCase, ports(1:2), [4 3]);
clear cleanup;
close_model(model);
end

function testManagedRepetitiveWorkspaceCanRestart(testCase)
model = 'gmp_mcb_unit_rc_memory';
cleanup = onCleanup(@() close_model(model));
new_system(model);
add_block('simulink/Sources/Constant', [model '/Error'], 'Value', '0.01');
add_block('simulink/Sources/Constant', [model '/Frequency'], 'Value', '50');
add_block(library_block('gmp_mcb_intrinsic_advance_repetitive_controller'), [model '/RC']);
set_param(model, 'SimulationCommand', 'update');
add_block('simulink/Sinks/Terminator', [model '/Out']);
add_line(model, 'Error/1', 'RC/1');
add_line(model, 'Frequency/1', 'RC/2');
add_line(model, 'RC/1', 'Out/1');
simulationInput = Simulink.SimulationInput(model).setModelParameter( ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', '1e-4', 'StopTime', '0.03');
sim(simulationInput);
sim(simulationInput);
clear cleanup;
close_model(model);
end

function block = pid_library_block()
block = library_block('gmp_mcb_intrinsic_continuous_pid');
end

function block = qpr_library_block()
block = library_block('gmp_mcb_intrinsic_discrete_resonant_qpr');
end

function block = library_block(functionName)
blocks = find_system('gmp_mcb_components', 'BlockType', 'S-Function');
names = get_param(blocks, 'FunctionName');
block = blocks{find(strcmp(names, functionName), 1)};
end

function close_model(model)
if bdIsLoaded(model), close_system(model, 0); end
end
