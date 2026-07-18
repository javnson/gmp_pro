function result = measure_pid_block(block, frequenciesHz)
% Backward-compatible PID measurement entry point.
if nargin < 2
    result = gmp_mcb.measure_component_block(block);
else
    result = gmp_mcb.measure_component_block(block, frequenciesHz);
end
end
