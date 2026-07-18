function apply_pid_mask(block, component)
% Apply the SISO PID mask to one generated S-Function block.
mask = Simulink.Mask.create(block);
mask.Type = 'GMP MATLAB Component Builder PID';
mask.Description = sprintf('%s\n\nThe S-Function inherits its execution sample time. fs is passed only to the GMP initializer.', ...
    char(component.description));
mask.addParameter('Type', 'popup', 'Name', 'init_method', ...
    'Prompt', 'Initialization method', ...
    'TypeOptions', {'Parallel gains (Kp, Ki, Kd)', 'Time constants (Kp, Ti, Td)'}, ...
    'Value', 'Parallel gains (Kp, Ki, Kd)');

parameters = component.parameters;
for index = 1:numel(parameters)
    parameter = parameters(index);
    prompt = char(parameter.label);
    if isfield(parameter, 'unit') && ~isempty(parameter.unit)
        prompt = sprintf('%s [%s]', prompt, char(parameter.unit));
    end
    mask.addParameter('Type', 'edit', 'Name', char(parameter.id), ...
        'Prompt', prompt, 'Value', num2str(parameter.default, 16));
end
mask.addParameter('Type', 'edit', 'Name', 'analysis_fs', ...
    'Prompt', 'Analysis/execution frequency [Hz]', 'Value', 'fs');

plotButton = mask.addDialogControl('pushbutton', 'plot_models');
plotButton.Prompt = 'Plot theoretical and implementation models';
plotButton.Callback = 'gmp_mcb.plot_pid_models(gcb);';
measureButton = mask.addDialogControl('pushbutton', 'measure_model');
measureButton.Prompt = 'Measure compiled S-Function response';
measureButton.Callback = 'gmp_mcb.measure_pid_block(gcb);';
mask.Display = sprintf(['disp(''%s'');\n' ...
    'port_label(''input'',1,''e'');\nport_label(''output'',1,''u'');'], char(component.display_name));
end
