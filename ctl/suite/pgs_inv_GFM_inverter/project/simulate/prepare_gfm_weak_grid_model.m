function model = prepare_gfm_weak_grid_model()
%PREPARE_GFM_WEAK_GRID_MODEL Build the reproducible weak-grid/load-step plant.
root = fileparts(mfilename('fullpath'));
source = 'DP_STD_MDL_DCAC_3ph_2level_gridconn';
model = 'DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid';
source_file = fullfile(root, [source '.slx']);
target_file = fullfile(root, [model '.slx']);

load_system(source_file);
source_cleanup = onCleanup(@() close_if_loaded(source)); %#ok<NASGU>
save_system(source, target_file);
close_system(source, 0);
load_system(target_file);
target_cleanup = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

% Increase the three phase grid impedance to represent a low short-circuit ratio.
grid_branches = {'Series RLC Branch','Series RLC Branch1','Series RLC Branch2'};
for k = 1:numel(grid_branches)
    set_param([model '/' grid_branches{k}], ...
        'BranchType', 'RL', 'Resistance', '0.20', 'Inductance', '3.0e-3');
end

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

% The additional grounded-Y constant-Z load closes after GFM takeover.
set_param(breaker, 'InitialState', 'open', 'External', 'off', ...
    'SwitchTimes', '[0.70]');
set_param(load_block, 'Configuration', 'Y (grounded)', ...
    'NominalVoltage', '56.6', 'NominalFrequency', '50', ...
    'ActivePower', '120', 'InductivePower', '0', ...
    'CapacitivePower', '0', 'LoadType', 'constant Z');

save_system(model, target_file);
end

function close_if_loaded(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
end
