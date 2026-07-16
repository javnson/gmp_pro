function configure_sinv_models()
%CONFIGURE_SINV_MODELS Bind all SINV commissioning plants to generated SDPE data.

root = fileparts(mfilename('fullpath'));
models = {'PGS_STD_SINV_MODEL_RLOAD', 'PGS_STD_SINV_MODEL_Grid', ...
          'PGS_STD_SINV_MODEL_Rectifier'};
common_init = 'sdpe_pgs_sinv_rc_common_settings_matlab_init.m';
simulate_init = 'sdpe_pgs_sinv_rc_simulate_settings_matlab_init.m';
callback = [ ...
    'gmp_model_file=get_param(bdroot,''FileName''); ' ...
    'if isempty(gmp_model_file), error(''GMP:SINV:ModelPath'', ' ...
    '''The SINV model must be saved before SDPE initialization.''); end; ' ...
    'gmp_model_dir=fileparts(gmp_model_file); ' ...
    'run(fullfile(gmp_model_dir,''..'',''..'',''sdpe_general'',''' common_init ''')); ' ...
    'run(fullfile(gmp_model_dir,''sdpe_mgr'',''' simulate_init ''')); ' ...
    'addpath(fullfile(gmp_model_dir,''..'',''..'',''..'',''..'',''..'',''tools'',' ...
    '''gmp_sil'',''udp_helper_v2'',''mdl_asio_helper'',''bin'',''x64'',''Debug'')); ' ...
    'clear gmp_model_file gmp_model_dir;'];

run(fullfile(root, '..', '..', 'sdpe_general', common_init));
run(fullfile(root, 'sdpe_mgr', simulate_init));

for k = 1:numel(models)
    model = models{k};
    load_system(fullfile(root, [model '.slx']));
    set_param(model, 'PreLoadFcn', '', 'PostLoadFcn', callback, ...
        'InitFcn', callback, ...
        'SolverType', 'Variable-step', 'Solver', 'VariableStepAuto', ...
        'FixedStep', 'auto');

    plant = [model '/Single Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];
    configure_plant_mask(plant);

    if getSimulinkBlockHandle([model '/Constant']) ~= -1
        set_param([model '/Constant'], 'Value', 'CTRL_DCBUS_VOLTAGE');
    end
    if strcmp(model, 'PGS_STD_SINV_MODEL_RLOAD')
        set_param([model '/Load'], 'BranchType', 'R', 'Resistance', 'SINV_RLOAD_OHM');
        set_param(model, 'StopTime', '2.0');
    else
        install_grid_source(model);
        if strcmp(model, 'PGS_STD_SINV_MODEL_Grid')
            set_param(model, 'StopTime', '3.0');
        else
            set_param([model '/Load1'], 'BranchType', 'R', 'Resistance', 'SINV_RECTIFIER_RLOAD_OHM');
            set_param(model, 'StopTime', '4.0');
        end
    end

    save_system(model);
    close_system(model);
end
fprintf('Configured and saved %d SINV models.\n', numel(models));
end

function configure_plant_mask(plant)
if ~strcmp(get_param(plant, 'LinkStatus'), 'none')
    set_param(plant, 'LinkStatus', 'none');
end
mask = Simulink.Mask.get(plant);
if isempty(mask)
    mask = Simulink.Mask.create(plant);
end
mask.Description = ['SDPE-configured single-phase bidirectional converter. ' ...
    'ADC bus order: IL, IDC, VDC, IC, VC, IG. PWM order: bridge legs L and N.'];

add_group(mask, 'PWM Configuration');
add_param(mask, 'fswitch', 'Switching frequency (Hz)', 'SINV_PWM_FREQUENCY_HZ', 'PWM Configuration');
add_param(mask, 'CMP_MAX', 'PWM compare full scale', 'CTRL_PWM_CMP_MAX + 1', 'PWM Configuration');
add_param(mask, 'Deadband_Upper', 'Upper deadband (counts)', 'CTRL_PWM_DEADBAND_CMP', 'PWM Configuration');
add_param(mask, 'Deadband_Lower', 'Lower deadband (counts)', 'CTRL_PWM_DEADBAND_CMP', 'PWM Configuration');
add_param(mask, 'phase_shift', 'Carrier phase shift (deg)', '0', 'PWM Configuration');

add_group(mask, 'Semiconductor Model');
add_param(mask, 'R_mosfet_on', 'MOSFET on resistance (Ohm)', 'SINV_MODEL_MOSFET_RON', 'Semiconductor Model');
add_param(mask, 'Lon', 'Switch series inductance (H)', '0', 'Semiconductor Model');
add_param(mask, 'Rd', 'Body-diode on resistance (Ohm)', 'SINV_MODEL_DIODE_RON', 'Semiconductor Model');
add_param(mask, 'Vf', 'Body-diode forward voltage (V)', 'SINV_MODEL_DIODE_VF', 'Semiconductor Model');
add_param(mask, 'Rs', 'Snubber resistance (Ohm)', '1e5', 'Semiconductor Model');
add_param(mask, 'Cs', 'Snubber capacitance (F)', 'inf', 'Semiconductor Model');

add_group(mask, 'AC Filter');
add_param(mask, 'ACF_L', 'Series inductance (H)', 'CTRL_AC_INDUCTANCE', 'AC Filter');
add_param(mask, 'ACF_L_ESR', 'Series resistance (Ohm)', 'CTRL_AC_RESISTANCE', 'AC Filter');
add_param(mask, 'ACF_C', 'Shunt capacitance (F)', 'SINV_FILTER_CAPACITANCE_F', 'AC Filter');
add_param(mask, 'ACF_C_ESR', 'Capacitor ESR (Ohm)', 'SINV_FILTER_CAP_ESR_OHM', 'AC Filter');
add_param(mask, 'ACF_C_EPR', 'Capacitor leakage resistance (Ohm)', '100e3', 'AC Filter');

add_group(mask, 'DC Link');
add_param(mask, 'DC_CAP', 'DC-link capacitance (F)', 'SINV_DC_CAPACITANCE_F', 'DC Link');
add_param(mask, 'DC_CAP_ESR', 'DC-link capacitor ESR (Ohm)', '0.05', 'DC Link');
add_param(mask, 'DC_CAP_EPR', 'DC-link leakage resistance (Ohm)', '10e3', 'DC Link');

add_group(mask, 'ADC Configuration');
add_param(mask, 'ADC_Resolution', 'ADC resolution (bits)', 'CTRL_ADC_RESOLUTION', 'ADC Configuration');

add_group(mask, 'Voltage Sensors');
add_param(mask, 'DC_Voltage_Gain', 'DC voltage sensitivity (V/V)', 'CTRL_DC_VOLTAGE_SENSITIVITY', 'Voltage Sensors');
add_param(mask, 'DC_VOLTAGE_ADC_REFERENCE', 'DC voltage ADC reference (V)', 'CTRL_ADC_VOLTAGE_REF', 'Voltage Sensors');
add_param(mask, 'DC_VOLTAGE_ADC_BIAS', 'DC voltage ADC bias (V)', 'CTRL_DC_VOLTAGE_BIAS', 'Voltage Sensors');
add_param(mask, 'DC_VOLTAGE_ADC_CUT_FREQ', 'DC voltage filter cutoff (Hz)', '1500', 'Voltage Sensors');
add_param(mask, 'AC_Voltage_Gain', 'AC voltage sensitivity (V/V)', 'CTRL_AC_VOLTAGE_SENSITIVITY', 'Voltage Sensors');
add_param(mask, 'AC_VOLTAGE_ADC_REFERENCE', 'AC voltage ADC reference (V)', 'CTRL_ADC_VOLTAGE_REF', 'Voltage Sensors');
add_param(mask, 'AC_VOLTAGE_ADC_BIAS', 'AC voltage ADC bias (V)', 'CTRL_AC_VOLTAGE_BIAS', 'Voltage Sensors');
add_param(mask, 'AC_VOLTAGE_ADC_CUT_FREQ', 'AC voltage filter cutoff (Hz)', '1500', 'Voltage Sensors');

add_group(mask, 'Current Sensors');
add_param(mask, 'AC_Current_Gain', 'AC current sensitivity (V/A)', 'CTRL_AC_CURRENT_SENSITIVITY', 'Current Sensors');
add_param(mask, 'AC_CURRENT_ADC_REFERENCE', 'AC current ADC reference (V)', 'CTRL_ADC_VOLTAGE_REF', 'Current Sensors');
add_param(mask, 'AC_CURRENT_ADC_BIAS', 'AC current ADC bias (V)', 'CTRL_AC_CURRENT_BIAS', 'Current Sensors');
add_param(mask, 'AC_CURRENT_ADC_INIT', 'AC current initial value', '0', 'Current Sensors');
add_param(mask, 'DC_BUS_CURRENT_SENSOR_R', 'DC shunt resistance (Ohm)', '0.02', 'Current Sensors');
add_param(mask, 'DC_Current_Gain', 'DC current amplifier gain', '20', 'Current Sensors');
add_param(mask, 'DC_CURRENT_ADC_REFERENCE', 'DC current ADC reference (V)', 'CTRL_ADC_VOLTAGE_REF', 'Current Sensors');
add_param(mask, 'DC_CURRENT_ADC_BIAS', 'DC current ADC bias (V)', 'CTRL_AC_CURRENT_BIAS', 'Current Sensors');
add_param(mask, 'DC_CURRENT_ADC_CUT_FREQ', 'DC current filter cutoff (Hz)', '1500', 'Current Sensors');
add_param(mask, 'CAP_CURRENT_SENSOR_R', 'Capacitor-current shunt (Ohm)', '0.02', 'Current Sensors');
add_param(mask, 'CAP_Current_Gain', 'Capacitor-current amplifier gain', '20', 'Current Sensors');
add_param(mask, 'CAP_CURRENT_ADC_REFERENCE', 'Capacitor-current ADC reference (V)', 'CTRL_ADC_VOLTAGE_REF', 'Current Sensors');
add_param(mask, 'CAP_CURRENT_ADC_BIAS', 'Capacitor-current ADC bias (V)', 'CTRL_AC_CURRENT_BIAS', 'Current Sensors');
add_param(mask, 'CAP_CURRENT_ADC_CUT_FREQ', 'Capacitor-current filter cutoff (Hz)', '1500', 'Current Sensors');
add_param(mask, 'GRID_CURRENT_SENSOR_R', 'Grid-current shunt (Ohm)', '0.02', 'Current Sensors');
add_param(mask, 'Grid_Current_Gain', 'Grid-current amplifier gain', 'CTRL_AC_CURRENT_SENSITIVITY / 0.02', 'Current Sensors');
add_param(mask, 'GRID_CURRENT_ADC_REFERENCE', 'Grid-current ADC reference (V)', 'CTRL_ADC_VOLTAGE_REF', 'Current Sensors');
add_param(mask, 'GRID_CURRENT_ADC_BIAS', 'Grid-current ADC bias (V)', 'CTRL_AC_CURRENT_BIAS', 'Current Sensors');
add_param(mask, 'GRID_CURRENT_ADC_CUT_FREQ', 'Grid-current filter cutoff (Hz)', '1500', 'Current Sensors');

mask.Display = sprintf(['disp(''GMP STD SINV/AFE\\nSDPE configured'');\n' ...
    'port_label(''input'',1,''Enable''); port_label(''input'',2,''PWM [L N]'');\n' ...
    'port_label(''output'',1,''ADC bus''); port_label(''output'',2,''Sample'');']);
end

function install_grid_source(model)
load_block = [model '/Load'];
source = [model '/Grid Voltage Source'];
sine = [model '/Grid Voltage Command'];
if getSimulinkBlockHandle(source) ~= -1
    return;
end

load_ph = get_param(load_block, 'PortHandles');
load_ports = [load_ph.LConn load_ph.RConn];
peers = zeros(size(load_ports));
for k = 1:numel(load_ports)
    line = get_param(load_ports(k), 'Line');
    src = get_param(line, 'SrcPortHandle');
    dst = get_param(line, 'DstPortHandle');
    candidates = [src dst(:)'];
    peers(k) = candidates(candidates ~= load_ports(k));
end
pos = get_param(load_block, 'Position');
delete_block(load_block);
template = [model '/Voltage Source1'];
if getSimulinkBlockHandle(template) == -1
    load_system('PGS_STD_SINV_MODEL_RLOAD');
    template = 'PGS_STD_SINV_MODEL_RLOAD/Voltage Source1';
end
add_block(template, source, 'Position', pos);
source_ph = get_param(source, 'PortHandles');
source_ports = [source_ph.LConn source_ph.RConn];
for k = 1:2
    add_line(model, source_ports(k), peers(k), 'autorouting', 'on');
end
add_block('simulink/Sources/Sine Wave', sine, ...
    'Amplitude', 'sqrt(2) * CTRL_GRID_VOLTAGE_RMS', ...
    'Frequency', '2*pi*CTRL_GRID_FREQUENCY', ...
    'SampleTime', '0', 'Position', pos + [-130 0 -130 0]);
sine_ph = get_param(sine, 'PortHandles');
source_ph = get_param(source, 'PortHandles');
add_line(model, sine_ph.Outport(1), source_ph.Inport(1), 'autorouting', 'on');
end

function add_group(mask, name)
id = [matlab.lang.makeValidName(name) '_Group'];
if isempty(mask.getDialogControl(id))
    mask.addDialogControl('Type', 'group', 'Name', id, 'Prompt', name);
end
end

function add_param(mask, name, prompt, value, group)
p = mask.getParameter(name);
if isempty(p)
    p = mask.addParameter('Type', 'edit', 'Name', name, 'Prompt', prompt, ...
        'Value', value, 'Evaluate', 'on', 'Tunable', 'off');
else
    p.Prompt = prompt;
    p.Value = value;
end
p.Container = [matlab.lang.makeValidName(group) '_Group'];
p.DialogControl.PromptLocation = 'left';
end
