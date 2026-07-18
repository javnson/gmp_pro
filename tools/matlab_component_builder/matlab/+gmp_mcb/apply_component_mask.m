function apply_component_mask(block, component)
% Create grouped parameter and analysis tabs for a generated SISO component.
mask = Simulink.Mask.create(block);
mask.Type = 'GMP MATLAB Component Builder SISO';
mask.Description = sprintf(['%s\n\nExecution sample time is inherited from the surrounding ' ...
    'solver, triggered subsystem, or function-call subsystem.'], char(component.description));

tabs = mask.addDialogControl('tabcontainer', 'mcb_tabs');
parameterTab = tabs.addDialogControl('tab', 'parameters_tab');
parameterTab.Prompt = 'Parameters';
analysisTab = tabs.addDialogControl('tab', 'analysis_tab');
analysisTab.Prompt = 'Simulation Analysis';

initGroup = parameterTab.addDialogControl('group', 'group_initialization_method');
initGroup.Prompt = 'Initialization method';
initializerLabels = cellstr(string({component.initializers.label}));
mask.addParameter('Type', 'popup', 'Name', 'init_method', ...
    'Prompt', 'Method', 'TypeOptions', initializerLabels, ...
    'Value', initializerLabels{1}, 'Container', initGroup.Name, 'Tunable', 'off');

parameters = component.parameters;
groupNames = strings(0);
groupControls = strings(0);
for index = 1:numel(parameters)
    parameter = parameters(index);
    groupName = "General";
    if isfield(parameter, 'group') && ~isempty(parameter.group), groupName = string(parameter.group); end
    groupIndex = find(groupNames == groupName, 1);
    if isempty(groupIndex)
        controlName = "group_" + string(matlab.lang.makeValidName(lower(groupName)));
        groupControl = parameterTab.addDialogControl('group', char(controlName));
        groupControl.Prompt = char(groupName);
        groupNames(end + 1) = groupName; %#ok<AGROW>
        groupControls(end + 1) = controlName; %#ok<AGROW>
        groupIndex = numel(groupNames);
    end
    prompt = char(parameter.label);
    if isfield(parameter, 'unit') && ~isempty(parameter.unit)
        prompt = sprintf('%s [%s]', prompt, char(parameter.unit));
    end
    mask.addParameter('Type', 'edit', 'Name', char(parameter.id), ...
        'Prompt', prompt, 'Value', num2str(parameter.default, 16), ...
        'Container', char(groupControls(groupIndex)), 'Tunable', 'off');
    if isfield(parameter, 'externalizable') && parameter.externalizable
        exposeName = ['expose_' char(parameter.id)];
        mask.addParameter('Type', 'checkbox', 'Name', exposeName, ...
            'Prompt', ['Use external input for ' char(parameter.label)], 'Value', 'off', ...
            'Container', char(groupControls(groupIndex)), 'Tunable', 'off', ...
            'Callback', sprintf('gmp_mcb.update_external_parameter(gcb,''%s'');', char(parameter.id)));
    end
end

analysis = component.analysis;
defaultExecutionFs = 10000;
fsIndex = find(strcmp(string({parameters.id}), "fs"), 1);
if ~isempty(fsIndex), defaultExecutionFs = parameters(fsIndex).default; end
mask.addParameter('Type', 'edit', 'Name', 'analysis_execution_fs', ...
    'Prompt', 'Standalone analysis execution frequency [Hz]', ...
    'Value', num2str(defaultExecutionFs, 16), ...
    'Container', analysisTab.Name);
mask.addParameter('Type', 'edit', 'Name', 'analysis_frequency_min', ...
    'Prompt', 'Minimum frequency [Hz]', 'Value', num2str(analysis.default_frequency_hz(1), 16), ...
    'Container', analysisTab.Name);
mask.addParameter('Type', 'edit', 'Name', 'analysis_frequency_max', ...
    'Prompt', 'Maximum frequency [Hz]', 'Value', num2str(analysis.default_frequency_hz(2), 16), ...
    'Container', analysisTab.Name);
mask.addParameter('Type', 'edit', 'Name', 'analysis_points', ...
    'Prompt', 'Frequency points', 'Value', num2str(analysis.default_points), ...
    'Container', analysisTab.Name);
mask.addParameter('Type', 'edit', 'Name', 'analysis_amplitude', ...
    'Prompt', 'Excitation amplitude', 'Value', num2str(analysis.excitation_amplitude, 16), ...
    'Container', analysisTab.Name);
mask.addParameter('Type', 'edit', 'Name', 'analysis_settling_periods', ...
    'Prompt', 'Settling periods', 'Value', num2str(analysis.settling_periods), ...
    'Container', analysisTab.Name);
mask.addParameter('Type', 'edit', 'Name', 'analysis_measurement_periods', ...
    'Prompt', 'Measurement periods', 'Value', num2str(analysis.measurement_periods), ...
    'Container', analysisTab.Name);

plotButton = analysisTab.addDialogControl('pushbutton', 'plot_models');
plotButton.Prompt = 'Plot ideal and implemented models';
plotButton.Callback = 'gmp_mcb.plot_component_models(gcb);';
measureButton = analysisTab.addDialogControl('pushbutton', 'measure_model');
measureButton.Prompt = 'Measure compiled S-Function response';
measureButton.Callback = 'gmp_mcb.measure_component_block(gcb);';

displayLines = {sprintf('disp(''%s'');', strrep(char(component.display_name), '''', '''''')), ...
    sprintf('port_label(''input'',1,''%s'');', char(component.inputs(1).label)), ...
    'mcb_port_index = 2;'};
for index = 1:numel(parameters)
    parameter = parameters(index);
    if isfield(parameter, 'externalizable') && parameter.externalizable
        displayLines{end + 1} = sprintf([ ...
            'if gmp_mcb.checkbox_code(expose_%s), port_label(''input'',mcb_port_index,''%s''); ' ...
            'mcb_port_index=mcb_port_index+1; end'], char(parameter.id), char(parameter.label)); %#ok<AGROW>
    end
end
displayLines{end + 1} = sprintf('port_label(''output'',1,''%s'');', char(component.outputs(1).label));
mask.Display = strjoin(displayLines, newline);
set_param(block, 'UserData', component, 'UserDataPersistent', 'on');
end
