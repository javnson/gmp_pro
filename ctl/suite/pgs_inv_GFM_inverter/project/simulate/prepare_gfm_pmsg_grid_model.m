function model = prepare_gfm_pmsg_grid_model()
%PREPARE_GFM_PMSG_GRID_MODEL Build the R2024b PMSG GFM test plant.
% The machine uses the Simulink/Simscape Electrical permanent-magnet
% synchronous-machine block operated as a generator. A large
% mechanical inertia and an initialized synchronous speed represent a
% generator whose rating is comfortably above the converter under test.
% Torque input is used instead of an ideal speed source so the SPS electrical
% network does not contain a current-source machine in series with an
% inductive weak-grid branch.
root = fileparts(mfilename('fullpath'));
source = 'DP_STD_MDL_DCAC_3ph_2level_gridconn';
model = 'DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid';
source_file = fullfile(root, [source '.slx']);
target_file = fullfile(root, [model '.slx']);
pmsg_active_power = 120;
pmsg_mechanical_speed = 2*pi*50/4;
pmsg_torque = pmsg_active_power/pmsg_mechanical_speed;
pmsg_voltage_pu = 0.33;
gfm_voltage_pu = 0.50;
converter_line_voltage_base_rms = 56.6;

load_system(source_file);
source_cleanup = onCleanup(@() close_if_loaded(source)); %#ok<NASGU>
save_system(source, target_file);
close_system(source, 0);
load_system(target_file);
target_cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

grid_branches = {'Series RLC Branch','Series RLC Branch1','Series RLC Branch2'};
measurements = {'Current Measurement','Current Measurement1','Current Measurement2'};
for k = 1:3
    set_param([model '/' grid_branches{k}], ...
        'BranchType', 'RL', 'Resistance', '0.20', 'Inductance', '10.0e-3');
end

% Replace the three ideal controlled sources with the built-in PM machine.
old_sources = {'Grid Source(A)','Grid Source(B)','Grid Source(B)1'};
for k = 1:numel(old_sources)
    block = [model '/' old_sources{k}];
    if ~isempty(find_system(model, 'SearchDepth', 1, 'Name', old_sources{k}))
        delete_block(block);
    end
end

machine = [model '/Large PMSG'];
prime_mover = [model '/Prime Mover Torque'];
measurement_term = [model '/PMSG Measurement Terminator'];
if isempty(find_system(model, 'SearchDepth', 1, 'Name', 'Large PMSG'))
    add_block('powerlib/Machines/Permanent Magnet Synchronous Machine', machine, ...
        'Position', [2380 100 2540 300]);
    add_block('simulink/Sources/Constant', prime_mover, ...
        'Value', num2str(pmsg_torque, 17), 'Position', [2200 110 2300 140]);
    add_block('simulink/Sinks/Terminator', measurement_term, ...
        'Position', [2620 130 2640 150]);
end

set_param(machine, 'NbPhases', '3', 'FluxDistribution', 'Sinusoidal', ...
    'RotorType', 'Round', 'MechanicalLoad', 'Torque Tm', 'PresetModel', 'No', ...
    'Resistance', '0.03', 'Inductance', '2.5e-3', ...
    'MachineConstant', 'Flux linkage established by magnets (V.s)', ...
    'Flux', '0.060', 'PolePairs', '4', ...
    'Mechanical', '[10.0 0.001 4 0]', ...
    'InitialConditions', '[78.53981633974483 0 0 0]');

machine_ports = get_param(machine, 'PortHandles');
prime_mover_ports = get_param(prime_mover, 'PortHandles');
term_ports = get_param(measurement_term, 'PortHandles');
if get_param(machine_ports.Inport(1), 'Line') == -1
    add_line(model, prime_mover_ports.Outport(1), machine_ports.Inport(1), ...
        'autorouting', 'on');
end
if get_param(machine_ports.Outport(1), 'Line') == -1
    add_line(model, machine_ports.Outport(1), term_ports.Inport(1), ...
        'autorouting', 'on');
end

grid_breaker = [model '/PMSG Grid Connection Breaker'];
add_block('powerlib/Elements/Three-Phase Breaker', grid_breaker, ...
    'Position', [2260 250 2350 340]);
set_param(grid_breaker, 'InitialState', 'open', 'External', 'off', ...
    'SwitchTimes', '[0.05 0.50]', 'BreakerResistance', '0.02', ...
    'SnubberResistance', '1.0e5', 'SnubberCapacitance', 'inf');
grid_breaker_ports = get_param(grid_breaker, 'PortHandles');
for phase = 1:3
    measurement_ports = get_param([model '/' measurements{phase}], 'PortHandles');
    add_line(model, machine_ports.LConn(phase), grid_breaker_ports.LConn(phase), ...
        'autorouting', 'on');
    add_line(model, grid_breaker_ports.RConn(phase), measurement_ports.RConn(1), ...
        'autorouting', 'on');
end

add_base_load(model, machine_ports, pmsg_active_power, ...
    converter_line_voltage_base_rms*pmsg_voltage_pu);
add_gfm_base_load(model, 30, converter_line_voltage_base_rms*gfm_voltage_pu);
add_load_step(model, converter_line_voltage_base_rms*gfm_voltage_pu);
save_system(model, target_file);
end

function add_base_load(model, machine_ports, active_power, nominal_voltage)
load_block = [model '/PMSG Matched Base Load'];
if isempty(find_system(model, 'SearchDepth', 1, 'Name', 'PMSG Matched Base Load'))
    add_block('powerlib/Elements/Three-Phase Parallel RLC Load', load_block, ...
        'Position', [2210 485 2310 565]);
    load_ports = get_param(load_block, 'PortHandles');
    for phase = 1:3
        add_line(model, machine_ports.LConn(phase), load_ports.LConn(phase), ...
            'autorouting', 'on');
    end
end
set_param(load_block, 'Configuration', 'Y (grounded)', ...
    'NominalVoltage', num2str(nominal_voltage, 17), 'NominalFrequency', '50', ...
    'ActivePower', num2str(active_power, 17), 'InductivePower', '0', ...
    'CapacitivePower', '0', 'LoadType', 'constant Z');
end

function add_gfm_base_load(model, active_power, nominal_voltage)
load_block = [model '/GFM Base Load'];
if isempty(find_system(model, 'SearchDepth', 1, 'Name', 'GFM Base Load'))
    add_block('powerlib/Elements/Three-Phase Parallel RLC Load', load_block, ...
        'Position', [2410 485 2510 565]);
    inverter = [model '/Three Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];
    inverter_ports = get_param(inverter, 'PortHandles');
    load_ports = get_param(load_block, 'PortHandles');
    for phase = 1:3
        add_line(model, inverter_ports.RConn(phase), load_ports.LConn(phase), ...
            'autorouting', 'on');
    end
end
set_param(load_block, 'Configuration', 'Y (grounded)', ...
    'NominalVoltage', num2str(nominal_voltage, 17), 'NominalFrequency', '50', ...
    'ActivePower', num2str(active_power, 17), 'InductivePower', '0', ...
    'CapacitivePower', '0', 'LoadType', 'constant Z');
end

function add_load_step(model, nominal_voltage)
breaker = [model '/GFM Load Step Breaker'];
load_block = [model '/GFM Three-Phase Load'];
if isempty(find_system(model, 'SearchDepth', 1, 'Name', 'GFM Load Step Breaker'))
    add_block('powerlib/Elements/Three-Phase Breaker', breaker, ...
        'Position', [2050 360 2140 450]);
    add_block('powerlib/Elements/Three-Phase Parallel RLC Load', load_block, ...
        'Position', [2210 365 2310 445]);
    inverter = [model '/Three Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];
    inverter_ports = get_param(inverter, 'PortHandles');
    breaker_ports = get_param(breaker, 'PortHandles');
    load_ports = get_param(load_block, 'PortHandles');
    for phase = 1:3
        add_line(model, inverter_ports.RConn(phase), breaker_ports.LConn(phase), ...
            'autorouting', 'on');
        add_line(model, breaker_ports.RConn(phase), load_ports.LConn(phase), ...
            'autorouting', 'on');
    end
end
set_param(breaker, 'InitialState', 'open', 'External', 'off', ...
    'SwitchTimes', '[0.70]');
set_param(load_block, 'Configuration', 'Y (grounded)', ...
    'NominalVoltage', num2str(nominal_voltage, 17), 'NominalFrequency', '50', ...
        'ActivePower', '120', 'InductivePower', '0', ...
    'CapacitivePower', '0', 'LoadType', 'constant Z');
end

function close_if_loaded(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
end
