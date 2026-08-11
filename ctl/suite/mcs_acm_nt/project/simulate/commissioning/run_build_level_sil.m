function summary = run_build_level_sil(stop_time_s, result_file)
%RUN_BUILD_LEVEL_SIL Run one ACIM BUILD_LEVEL commissioning capture.
%
%   SUMMARY = RUN_BUILD_LEVEL_SIL(STOP_TIME_S) attaches temporary logging
%   blocks to all sixteen controller monitor channels and to the induction
%   machine mechanical measurement bus.  The model is closed without being
%   saved, so this helper never modifies MCS_STD_ACM_MODEL.slx.
%
%   SUMMARY = RUN_BUILD_LEVEL_SIL(STOP_TIME_S, RESULT_FILE) also writes the
%   summary table as CSV.  The SIL executable must already be running.
%
%   Controller monitor channel definitions are maintained in:
%     xplt/xplt.ctl_interface.h

%   Angle convention: every controller angle channel uses electrical-turn
%   per-unit, where 1 pu is one complete revolution (2*pi rad).


%% Argument validation

arguments
    stop_time_s (1, 1) double {mustBePositive} = 0.05
    result_file (1, 1) string = ""
end

model = "MCS_STD_ACM_MODEL";
model_file = fullfile(fileparts(fileparts(mfilename("fullpath"))), model + ".slx");
assert(isfile(model_file), "ACIM SIL model was not found: %s", model_file);


%% Load the model and attach temporary loggers

load_system(model_file);
cleanup_model = onCleanup(@() close_system(model, 0));
set_param(model, "StopTime", string(stop_time_s));

signal_map = attach_controller_monitors(model);
signal_map = [signal_map; attach_machine_monitors(model)]; %#ok<AGROW>


%% Run SIL simulation and summarize each captured signal

sim_output = sim(model, "ReturnWorkspaceOutputs", "on");
summary = table('Size', [height(signal_map), 6], ...
    'VariableTypes', {'string', 'string', 'double', 'double', 'double', 'double'}, ...
    'VariableNames', {'Variable', 'Signal', 'Samples', 'Final', 'Minimum', 'Maximum'});

for index = 1:height(signal_map)
    value = sim_output.get(signal_map.Variable(index));
    data = double(value.Data(:));
    summary.Variable(index) = signal_map.Variable(index);
    summary.Signal(index) = signal_map.Signal(index);
    summary.Samples(index) = numel(data);
    summary.Final(index) = data(end);
    summary.Minimum(index) = min(data);
    summary.Maximum(index) = max(data);
end

disp(summary);

if strlength(result_file) > 0
    result_directory = fileparts(result_file);
    if strlength(result_directory) > 0 && ~isfolder(result_directory)
        mkdir(result_directory);
    end
    writetable(summary, result_file);
end

clear cleanup_model;
end


%% Controller monitor bus

function signal_map = attach_controller_monitors(model)
selectors = find_system(model, "SearchDepth", 2, "BlockType", "BusSelector");
variables = strings(0, 1);
signals = strings(0, 1);

for selector_index = 1:numel(selectors)
    output_signals = string(split(get_param(selectors{selector_index}, "OutputSignals"), ","));
    if ~all(startsWith(output_signals, "Monitor CH"))
        continue;
    end

    source_ports = get_param(selectors{selector_index}, "PortHandles");
    for port_index = 1:numel(output_signals)
        channel = sscanf(output_signals(port_index), "Monitor CH%d");
        variable = sprintf("sil_ch%02d", channel);
        add_timeseries_logger(model, source_ports.Outport(port_index), variable);
        variables(end + 1, 1) = string(variable); %#ok<AGROW>
        signals(end + 1, 1) = output_signals(port_index); %#ok<AGROW>
    end
end

signal_map = table(variables, signals, 'VariableNames', {'Variable', 'Signal'});
signal_map = sortrows(signal_map, "Variable");
assert(height(signal_map) == 16, ...
    "Expected sixteen controller monitor channels, but found %d.", height(signal_map));
end


%% Machine mechanical measurement bus

function signal_map = attach_machine_monitors(model)
selectors = find_system(model, "SearchDepth", 2, "BlockType", "BusSelector");
machine_selector = "";

for selector_index = 1:numel(selectors)
    output_signals = string(get_param(selectors{selector_index}, "OutputSignals"));
    if contains(output_signals, "Mechanical.Rotor speed (wm)")
        machine_selector = string(selectors{selector_index});
        break;
    end
end

assert(strlength(machine_selector) > 0, "The induction-machine measurement bus was not found.");
output_signals = [ ...
    "Mechanical.Rotor angle thetam (rad)";
    "Stator measurements.Stator current  is_q (A)";
    "Stator measurements.Stator current is_d (A)";
    "Mechanical.Rotor speed (wm)";
    "Mechanical.Electromagnetic torque Te (N*m)";
    "Rotor measurements.Rotor flux phir_q (V s)";
    "Rotor measurements.Rotor flux phir_d (V s)";
    "Stator measurements.Stator voltage vs_q (V)";
    "Stator measurements.Stator voltage vs_d (V)"];
set_param(machine_selector, "OutputSignals", strjoin(output_signals, ","));
source_ports = get_param(machine_selector, "PortHandles");

variables = ["sil_rotor_angle_rad"; "sil_stator_iq_a"; "sil_stator_id_a"; ...
    "sil_rotor_speed_rad_s"; "sil_torque_nm"; "sil_rotor_flux_q_vs"; ...
    "sil_rotor_flux_d_vs"; "sil_stator_vq_v"; "sil_stator_vd_v"];
signal_map = table(variables, output_signals, 'VariableNames', {'Variable', 'Signal'});

for port_index = 1:numel(output_signals)
    add_timeseries_logger(model, source_ports.Outport(port_index), variables(port_index));
end
end


%% Temporary To Workspace block helper

function add_timeseries_logger(model, source_port, variable)
block = model + "/CommissioningLog_" + variable;
add_block("simulink/Sinks/To Workspace", block, ...
    "VariableName", variable, ...
    "SaveFormat", "Timeseries", ...
    "MaxDataPoints", "inf", ...
    "Position", [1500, 40, 1660, 70]);
destination_ports = get_param(block, "PortHandles");
add_line(model, source_port, destination_ports.Inport, "autorouting", "on");
end
