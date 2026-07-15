function configure_fsbb_model()
%CONFIGURE_FSBB_MODEL Apply the reproducible SDPE mask configuration.

model = 'MCS_STD_FSBB_MODEL';
root = fileparts(mfilename('fullpath'));
load_system(fullfile(root, [model '.slx']));
fsbb = [model '/GMP STD FSBB Module'];

% Make both generated SDPE variables and the UDP S-function discoverable.
% PreLoadFcn runs before block parameters are evaluated, so use the resolved
% project location captured when this reproducible configurator is run.
sdpe_dir = strrep(fullfile(root, 'sdpe_mgr'), '\', '/');
mex_dir = strrep(fullfile(root, '..', '..', '..', '..', '..', 'tools', ...
    'gmp_sil', 'udp_helper_v2', 'mdl_asio_helper', 'bin', 'x64', 'Debug'), '\', '/');
preload = sprintf(['addpath(''%s''); addpath(''%s''); ' ...
    'evalin(''base'',''sdpe_dps_fsbb_simulate_settings_matlab_init;'');'], sdpe_dir, mex_dir);
set_param(model, 'PreLoadFcn', preload);
set_param(model, 'InitFcn', preload);
addpath(fullfile(root, 'sdpe_mgr'));
sdpe_dps_fsbb_simulate_settings_matlab_init;

% Correct the sensor name without changing its electrical connections.
old_iin = [fsbb '/Output Current Measurement1'];
if getSimulinkBlockHandle(old_iin) ~= -1
    set_param(old_iin, 'Name', 'Input Current Measurement');
end

% Build a mask whose values are expressions exported by SDPE.
mask = Simulink.Mask.get(fsbb);
if ~isempty(mask)
    mask.delete;
end
mask = Simulink.Mask.create(fsbb);
mask.Description = ['SDPE-configured four-switch buck-boost plant. ' ...
    'ADC output order: Vin, Vout, IL, Iout (Boost side), Iin (Buck side).'];

add_group(mask, 'PWM');
add_param(mask, 'Switching_Frequency', 'Switching frequency (Hz)', 'PWM_FREQ', 'PWM');
add_param(mask, 'PWM_Compare_Max', 'PWM compare maximum', 'CTRL_PWM_CMP_MAX', 'PWM');
add_param(mask, 'Deadband_Compare', 'Deadband (compare counts)', 'CTRL_PWM_DEADBAND_CMP', 'PWM');
add_param(mask, 'Phase_Shift', 'Phase shift (deg)', '0', 'PWM');

add_group(mask, 'Power Stage');
add_param(mask, 'Main_Inductance', 'Main inductance (H)', 'FSBB_PARAM_L', 'Power Stage');
add_param(mask, 'Inductor_ESR', 'Inductor ESR (Ohm)', 'FSBB_PARAM_L_ESR', 'Power Stage');
add_param(mask, 'Input_Capacitance', 'Input capacitance (F)', 'FSBB_PARAM_CIN', 'Power Stage');
add_param(mask, 'Input_Capacitor_ESR', 'Input capacitor ESR (Ohm)', 'FSBB_PARAM_CIN_ESR', 'Power Stage');
add_param(mask, 'Output_Capacitance', 'Output capacitance (F)', 'FSBB_PARAM_COUT', 'Power Stage');
add_param(mask, 'Output_Capacitor_ESR', 'Output capacitor ESR (Ohm)', 'FSBB_PARAM_COUT_ESR', 'Power Stage');
add_param(mask, 'MOSFET_Ron', 'MOSFET on resistance (Ohm)', 'FSBB_MODEL_MOSFET_RON', 'Power Stage');
add_param(mask, 'Switch_Lon', 'Switch series inductance (H)', 'FSBB_MODEL_SWITCH_LON', 'Power Stage');
add_param(mask, 'Diode_Ron', 'Body-diode on resistance (Ohm)', 'FSBB_MODEL_DIODE_RON', 'Power Stage');
add_param(mask, 'Diode_Vf', 'Body-diode forward voltage (V)', 'FSBB_MODEL_DIODE_VF', 'Power Stage');
add_param(mask, 'Snubber_R', 'Snubber resistance (Ohm)', 'FSBB_MODEL_SNUBBER_R', 'Power Stage');
add_param(mask, 'Snubber_C', 'Snubber capacitance (F)', 'FSBB_MODEL_SNUBBER_C', 'Power Stage');

add_group(mask, 'ADC and Sensors');
add_param(mask, 'ADC_Bits', 'ADC resolution (bits)', 'CTRL_ADC_RESOLUTION', 'ADC and Sensors');
add_param(mask, 'ADC_Reference', 'ADC reference (V)', 'CTRL_ADC_VOLTAGE_REF', 'ADC and Sensors');
add_param(mask, 'Vin_Sensor_Gain', 'Vin sensitivity (V/V)', 'CTRL_FSBB_VIN_SENSITIVITY', 'ADC and Sensors');
add_param(mask, 'Vin_Sensor_Bias', 'Vin bias (V)', 'CTRL_FSBB_VIN_BIAS', 'ADC and Sensors');
add_param(mask, 'Vout_Sensor_Gain', 'Vout sensitivity (V/V)', 'CTRL_FSBB_VOUT_SENSITIVITY', 'ADC and Sensors');
add_param(mask, 'Vout_Sensor_Bias', 'Vout bias (V)', 'CTRL_FSBB_VOUT_BIAS', 'ADC and Sensors');
add_param(mask, 'IL_Sensor_Gain', 'IL sensitivity (V/A)', 'CTRL_FSBB_IL_SENSITIVITY', 'ADC and Sensors');
add_param(mask, 'IL_Sensor_Bias', 'IL bias (V)', 'CTRL_FSBB_IL_BIAS', 'ADC and Sensors');
add_param(mask, 'Iout_Sensor_Gain', 'Iout sensitivity (V/A)', 'CTRL_FSBB_IOUT_SENSITIVITY', 'ADC and Sensors');
add_param(mask, 'Iout_Sensor_Bias', 'Iout bias (V)', 'CTRL_FSBB_IOUT_BIAS', 'ADC and Sensors');
add_param(mask, 'Iin_Sensor_Gain', 'Iin sensitivity (V/A)', 'CTRL_FSBB_IIN_SENSITIVITY', 'ADC and Sensors');
add_param(mask, 'Iin_Sensor_Bias', 'Iin bias (V)', 'CTRL_FSBB_IIN_BIAS', 'ADC and Sensors');
add_param(mask, 'ADC_Initial_Value', 'Sample-and-hold initial value', '0', 'ADC and Sensors');

mask.Display = sprintf(['disp(''GMP STD FSBB\\nSDPE configured'');\n' ...
    'port_label(''input'',1,''Enable''); port_label(''input'',2,''PWM [Buck Boost]'');\n' ...
    'port_label(''output'',1,''Sample''); port_label(''output'',2,''ADC [Vin Vout IL Iout Iin]'');']);

% Bind the existing phase and measurement masks to the parent SDPE mask.
for phase = {'Buck Phase', 'Boost Phase'}
    b = [fsbb '/' phase{1}];
    set_param(b, 'fswitch', 'Switching_Frequency', 'CMP_MAX', 'PWM_Compare_Max', ...
        'Deadband_Upper', 'Deadband_Compare', 'Deadband_Lower', 'Deadband_Compare', ...
        'phase_shift', 'Phase_Shift', 'R_mosfet_on', 'MOSFET_Ron', ...
        'Lon', 'Switch_Lon', 'Rd', 'Diode_Ron', 'Vf', 'Diode_Vf', ...
        'Rs', 'Snubber_R', 'Cs', 'Snubber_C');
end

bind_sensor([fsbb '/Input Voltage Measurement'], 'Vin_Sensor_Gain', 'Vin_Sensor_Bias');
bind_sensor([fsbb '/Output Voltage Measurement'], 'Vout_Sensor_Gain', 'Vout_Sensor_Bias');
bind_sensor([fsbb '/Inductor Current Measurement'], 'IL_Sensor_Gain', 'IL_Sensor_Bias');
bind_sensor([fsbb '/Output Current Measurement'], 'Iout_Sensor_Gain', 'Iout_Sensor_Bias');
bind_sensor([fsbb '/Input Current Measurement'], 'Iin_Sensor_Gain', 'Iin_Sensor_Bias');

% The original voltage Goto tags were crossed even though the electrical
% measurement blocks themselves are connected to the correct nodes.
set_param([fsbb '/Goto3'], 'GotoTag', 'InputVoltage');
set_param([fsbb '/Goto4'], 'GotoTag', 'OutputVoltage');

set_param([fsbb '/Main Inductor'], 'Resistance', 'Inductor_ESR', 'Inductance', 'Main_Inductance');
set_param([fsbb '/Main Input Capacitor'], 'Resistance', 'Input_Capacitor_ESR', 'Capacitance', 'Input_Capacitance');
set_param([fsbb '/Main Output Capacitor'], 'Resistance', 'Output_Capacitor_ESR', 'Capacitance', 'Output_Capacitance');

% Controller contract: raw ADC[0..4] = Vin, Vout, IL, Iout, Iin.
mux = [fsbb '/Mux3'];
ph = get_param(mux, 'PortHandles');
for k = 1:numel(ph.Inport)
    line = get_param(ph.Inport(k), 'Line');
    if line ~= -1
        delete_line(line);
    end
end
sources = {'From', 'From4', 'From3', 'From1', 'From2'};
for k = 1:numel(sources)
    src_ph = get_param([fsbb '/' sources{k}], 'PortHandles');
    add_line(fsbb, src_ph.Outport(1), ph.Inport(k), 'autorouting', 'on');
end

% Keep the surrounding architecture, but bind source and load to SDPE values.
set_param([model '/Constant'], 'Value', 'FSBB_INPUT_VOLTAGE_NOMINAL');
set_param([model '/Series RLC Branch'], 'Resistance', 'FSBB_PARAM_RLOAD_MIN');
save_system(model);
close_system(model);
fprintf('Configured and saved %s.\n', fullfile(root, [model '.slx']));
end

function add_group(mask, name)
% Section headings are carried in parameter prompts for compatibility with
% MATLAB releases that expose different mask-dialog container APIs.
mask = mask; %#ok<NASGU>
name = name; %#ok<NASGU>
end

function add_param(mask, name, prompt, value, group)
mask.addParameter('Type', 'edit', 'Name', name, 'Prompt', [group ': ' prompt], ...
    'Value', value, 'Evaluate', 'on', 'Tunable', 'off');
end

function bind_sensor(block, gain, bias)
set_param(block, 'ADC_GAIN', gain, 'ADC_BIT', 'ADC_Bits', ...
    'ADC_REFERENCE', 'ADC_Reference', 'ADC_BIAS', bias, ...
    'ADC_INIT', 'ADC_Initial_Value');
end
