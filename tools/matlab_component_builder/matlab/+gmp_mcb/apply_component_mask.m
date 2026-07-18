function apply_component_mask(block, component)
% Create grouped parameter and analysis tabs for a generated component.
mask = Simulink.Mask.create(block);
mask.Type = 'GMP MATLAB Component Builder';
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
defaultBias = 0;
if isfield(analysis, 'excitation_bias'), defaultBias = analysis.excitation_bias; end
mask.addParameter('Type', 'edit', 'Name', 'analysis_bias', ...
    'Prompt', 'Excitation bias / operating point', 'Value', num2str(defaultBias, 16), ...
    'Container', analysisTab.Name);
if numel(component.inputs) > 1
    inputLabels = cellstr(string({component.inputs.label}));
    mask.addParameter('Type', 'popup', 'Name', 'analysis_input_port', ...
        'Prompt', 'Excitation input port', 'TypeOptions', inputLabels, ...
        'Value', inputLabels{1}, 'Container', analysisTab.Name, 'Tunable', 'off');
    for index = 1:numel(component.inputs)
        input = component.inputs(index);
        defaultOperating = 0;
        if isfield(analysis, 'default_operating_values') && ...
                numel(analysis.default_operating_values) >= index
            defaultOperating = analysis.default_operating_values(index);
        end
        operatingPrompt = ['Steady operating value: ' char(input.label)];
        if isfield(analysis, 'settling_reference_input') && ...
                strcmp(char(analysis.settling_reference_input), char(input.id))
            operatingPrompt = ['Locked frequency: ' char(input.label) ' [Hz]'];
        end
        mask.addParameter('Type', 'edit', ...
            'Name', ['analysis_operating_' char(input.id)], ...
            'Prompt', operatingPrompt, ...
            'Value', num2str(defaultOperating, 16), 'Container', analysisTab.Name);
    end
end
if numel(component.outputs) > 1
    outputLabels = cellstr(string({component.outputs.label}));
    mask.addParameter('Type', 'popup', 'Name', 'analysis_output_port', ...
        'Prompt', 'Measured output port', 'TypeOptions', outputLabels, ...
        'Value', outputLabels{1}, 'Container', analysisTab.Name, 'Tunable', 'off');
end
mask.addParameter('Type', 'edit', 'Name', 'analysis_settling_periods', ...
    'Prompt', settling_prompt(analysis), 'Value', num2str(analysis.settling_periods), ...
    'Container', analysisTab.Name);
mask.addParameter('Type', 'edit', 'Name', 'analysis_measurement_periods', ...
    'Prompt', 'Measurement periods', 'Value', num2str(analysis.measurement_periods), ...
    'Container', analysisTab.Name);

if any(strcmp(char(component.template), {'pid_siso_v2', 'resonant_siso_v1'}))
    plotButton = analysisTab.addDialogControl('pushbutton', 'plot_models');
    plotButton.Prompt = 'Plot ideal and implemented models';
    plotButton.Callback = 'gmp_mcb.plot_component_models(gcb);';
end
measureButton = analysisTab.addDialogControl('pushbutton', 'measure_model');
measureButton.Prompt = 'Measure and plot simulated frequency response';
measureButton.Callback = 'gmp_mcb.measure_component_block(gcb);';

displayLines = {sprintf('disp(''%s'');', strrep(char(component.display_name), '''', '''''')), ...
    'mcb_ports = get_param(gcb,''Ports'');'};
for index = 1:numel(component.inputs)
    displayLines{end + 1} = sprintf('if mcb_ports(1)>=%d, port_label(''input'',%d,''%s''); end', ...
        index, ...
        index, char(component.inputs(index).label)); %#ok<AGROW>
end
displayLines{end + 1} = sprintf('mcb_port_index = %d;', numel(component.inputs) + 1);
for index = 1:numel(parameters)
    parameter = parameters(index);
    if isfield(parameter, 'externalizable') && parameter.externalizable
        displayLines{end + 1} = sprintf([ ...
            'if gmp_mcb.checkbox_code(expose_%s) && mcb_ports(1)>=mcb_port_index, port_label(''input'',mcb_port_index,''%s''); ' ...
            'mcb_port_index=mcb_port_index+1; end'], char(parameter.id), char(parameter.label)); %#ok<AGROW>
    end
end
for index = 1:numel(component.outputs)
    displayLines{end + 1} = sprintf('if mcb_ports(2)>=%d, port_label(''output'',%d,''%s''); end', ...
        index, index, char(component.outputs(index).label)); %#ok<AGROW>
end
mask.Display = strjoin(displayLines, newline);
set_param(block, 'UserData', component, 'UserDataPersistent', 'on');
end

function prompt = settling_prompt(analysis)
prompt = 'Settling periods';
if isfield(analysis, 'settling_reference_input')
    prompt = 'Pre-learning periods at locked frequency';
end
end
