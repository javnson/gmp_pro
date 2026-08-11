function run_sil_single_model_child(serverPort, clientPort, connectionIdText, instance)
%RUN_SIL_SINGLE_MODEL_CHILD One-process half of the two-model SIL test.

bytes = uint8(sscanf(char(connectionIdText), '%2x').');
assert(numel(bytes) == 16, 'Connection ID must contain 16 bytes.');
model = sprintf('gmp_sil_child_%d', instance);
new_system(model);
cleanup = onCleanup(@() close_system(model, 0));
add_block('simulink/Sources/Constant', [model '/input'], ...
    'Value', sprintf('uint8(%d:%d)', instance, instance + 31), ...
    'OutDataTypeStr', 'uint8', 'SampleTime', '1e-4');
add_block('simulink/User-Defined Functions/S-Function', [model '/core'], ...
    'FunctionName', 'GMP_SIL_Core', ...
    'Parameters', sprintf(['1, 3, [1 24], 1, 3, [1 32], 1, 0, ', ...
        '[127 0 0 1], [%d %d], %s'], ...
        serverPort, clientPort, mat2str(double(bytes))));
add_block('simulink/Sinks/Terminator', [model '/output']);
add_line(model, 'input/1', 'core/1');
add_line(model, 'core/1', 'output/1');
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1e-4', 'StopTime', '4e-4');
sim(model, 'SimulationMode', 'normal');
clear cleanup;
close_system(model, 0);
end
