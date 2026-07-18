function models = plot_pid_models(block, frequenciesHz)
% Backward-compatible PID plotting entry point.
if nargin < 2
    models = gmp_mcb.plot_component_models(block);
else
    models = gmp_mcb.plot_component_models(block, frequenciesHz);
end
end
