function metrics = run_gfm_validation(build_level, stop_time, label)
%RUN_GFM_VALIDATION Run one UDP SIL commissioning case and save metrics.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:5)}
    stop_time (1,1) double {mustBePositive} = 2.0
    label (1,:) char = ''
end

root = fileparts(mfilename('fullpath'));
model = model_for_level(build_level);
load_system(fullfile(root, [model '.slx']));
cleanup_model = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
expose_extended_monitors(model);

add_monitor_logger(model, 1, 'ia_pu', 1);
add_monitor_logger(model, 2, 'ib_pu', 2);
add_monitor_logger(model, 5, 'v_alpha_pu', 3);
add_monitor_logger(model, 6, 'v_beta_pu', 4);
add_monitor_logger(model, 7, 'pll_error_pu', 5);
add_monitor_logger(model, 8, 'angle_pu', 6);
add_monitor_logger(model, 9, 'voltage_command_d_pu', 7);
add_monitor_logger(model, 10, 'frequency_command_hz', 8);
add_monitor_logger(model, 11, 'id_pu', 9);
add_monitor_logger(model, 12, 'iq_pu', 10);
add_monitor_logger(model, 13, 'vd_pu', 11);
add_monitor_logger(model, 14, 'vq_pu', 12);
add_monitor_logger(model, 15, 'current_command_d_pu', 13);
add_monitor_logger(model, 16, 'transition_blend', 14);
add_block_logger(model, [model '/From11'], 1, 'output_enable', 15);

out = run_loaded_model(root, model, build_level, stop_time);
names = {'ia_pu','ib_pu','v_alpha_pu','v_beta_pu','pll_error_pu', ...
    'angle_pu','voltage_command_d_pu','frequency_command_hz', ...
    'id_pu','iq_pu','vd_pu','vq_pu', ...
    'current_command_d_pu','transition_blend','output_enable'};
signals = struct;
for k = 1:numel(names)
    signals.(names{k}) = out.get(names{k});
end

tail_time = 0.80 * stop_time;
metrics = struct;
metrics.build_level = build_level;
metrics.model = model;
metrics.stop_time_s = stop_time;
metrics.output_enable_final = last_value(signals.output_enable);
metrics.phase_current_rms_pu = tail_rms(signals.ia_pu, tail_time);
metrics.phase_current_peak_pu = tail_peak(signals.ia_pu, tail_time);
metrics.voltage_alpha_rms_pu = tail_rms(signals.v_alpha_pu, tail_time);
metrics.voltage_beta_rms_pu = tail_rms(signals.v_beta_pu, tail_time);
metrics.voltage_d_mean_pu = tail_mean(signals.vd_pu, tail_time);
metrics.voltage_q_rms_pu = tail_rms(signals.vq_pu, tail_time);
metrics.current_d_mean_pu = tail_mean(signals.id_pu, tail_time);
metrics.current_q_mean_pu = tail_mean(signals.iq_pu, tail_time);
metrics.pll_error_rms_pu = tail_rms(signals.pll_error_pu, tail_time);
metrics.current_command_peak_pu = tail_peak(signals.current_command_d_pu, 0);
metrics.current_command_max_step_pu = max_step(signals.current_command_d_pu);
metrics.transition_blend_final = last_value(signals.transition_blend);
metrics.frequency_hz = angle_frequency(signals.angle_pu, tail_time);
metrics.voltage_command_d_mean_pu = tail_mean(signals.voltage_command_d_pu, tail_time);
metrics.frequency_command_mean_hz = tail_mean(signals.frequency_command_hz, tail_time);
metrics.finite = all(structfun(@finite_scalar, rmfield(metrics, {'model'})));

if build_level == 1
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.voltage_alpha_rms_pu > 0.20;
elseif build_level == 2
    metrics.current_tracking_error_pu = hypot( ...
        metrics.current_d_mean_pu - 0.10, metrics.current_q_mean_pu - 0.10);
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.current_tracking_error_pu < 0.05;
elseif build_level == 3
    metrics.voltage_tracking_error_pu = abs(metrics.voltage_d_mean_pu - 0.50);
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.voltage_tracking_error_pu < 0.08 && ...
        metrics.current_command_peak_pu <= 0.805;
elseif build_level == 4
    metrics.current_tracking_error_pu = hypot( ...
        metrics.current_d_mean_pu - 0.10, metrics.current_q_mean_pu);
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.current_tracking_error_pu < 0.05 && ...
        metrics.pll_error_rms_pu < 0.02;
else
    pmsg_inertia = 10.0;
    pmsg_mechanical_speed = 2*pi*50/4;
    pmsg_input_power = 120.0;
    converter_v_phase_peak_base = 80/sqrt(3);
    converter_i_phase_peak_limit = 0.8*10;
    metrics.voltage_tracking_error_pu = abs( ...
        metrics.voltage_d_mean_pu - metrics.voltage_command_d_mean_pu);
    metrics.pre_load_current_rms_pu = window_rms(signals.ia_pu, 0.56, 0.68);
    metrics.post_load_current_rms_pu = window_rms(signals.ia_pu, 0.90, stop_time);
    metrics.load_current_step_pu = ...
        metrics.post_load_current_rms_pu - metrics.pre_load_current_rms_pu;
    metrics.pmsg_kinetic_energy_j = ...
        0.5*pmsg_inertia*pmsg_mechanical_speed^2;
    metrics.pmsg_inertia_constant_s_at_120w = ...
        metrics.pmsg_kinetic_energy_j/pmsg_input_power;
    metrics.pmsg_worst_case_rocof_hz_per_s_at_120w = ...
        4/(2*pi)*pmsg_input_power/(pmsg_inertia*pmsg_mechanical_speed);
    metrics.converter_current_limit_peak_a = converter_i_phase_peak_limit;
    metrics.converter_active_power_limit_w_at_voltage_command = ...
        1.5*converter_v_phase_peak_base* ...
        metrics.voltage_command_d_mean_pu*converter_i_phase_peak_limit;
    metrics.base_load_active_power_w = 30.0;
    metrics.load_step_active_power_w = 120.0;
    metrics.total_load_active_power_w = ...
        metrics.base_load_active_power_w + metrics.load_step_active_power_w;
    metrics.total_load_capacity_utilization = ...
        metrics.total_load_active_power_w/ ...
        metrics.converter_active_power_limit_w_at_voltage_command;
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.transition_blend_final > 0.99 && ...
        metrics.voltage_tracking_error_pu < 0.10 && ...
        metrics.voltage_q_rms_pu < 0.08 && ...
        metrics.frequency_hz > 47 && metrics.frequency_hz < 53 && ...
        metrics.current_command_max_step_pu < 0.08 && ...
        metrics.current_command_peak_pu <= 0.805 && ...
        metrics.phase_current_peak_pu <= 0.85 && ...
        metrics.load_current_step_pu > 0.02 && ...
        metrics.total_load_capacity_utilization < 0.60;
end

result_dir = fullfile(root, 'validation');
if ~isfolder(result_dir), mkdir(result_dir); end
if isempty(label), label = sprintf('build_level_%d', build_level); end
stem = matlab.lang.makeValidName(label);

fig = figure('Visible','off','Color','w','Position',[100 100 1200 900]);
tiledlayout(fig, 4, 1, 'TileSpacing','compact','Padding','compact');
nexttile;
plot(signals.vd_pu.Time, signals.vd_pu.Data, ...
    signals.vq_pu.Time, signals.vq_pu.Data, ...
    signals.voltage_command_d_pu.Time, signals.voltage_command_d_pu.Data, ...
    'LineWidth', 1.0);
grid on; ylabel('Voltage (pu)'); legend('v_d','v_q','v_d^*');
title(strrep(label, '_', ' '));
nexttile;
plot(signals.ia_pu.Time, signals.ia_pu.Data, ...
    signals.ib_pu.Time, signals.ib_pu.Data, 'LineWidth', 1.0);
grid on; ylabel('Current (pu)'); legend('i_a','i_b');
nexttile;
plot(signals.current_command_d_pu.Time, signals.current_command_d_pu.Data, ...
    signals.transition_blend.Time, signals.transition_blend.Data, 'LineWidth', 1.0);
grid on; ylabel('Transfer'); legend('i_d^*','GFM blend');
nexttile;
plot(signals.angle_pu.Time, signals.angle_pu.Data, ...
    signals.pll_error_pu.Time, signals.pll_error_pu.Data, 'LineWidth', 1.0);
grid on; ylabel('Sync'); xlabel('Time (s)'); legend('angle (pu)','PLL error');
exportgraphics(fig, fullfile(result_dir, [stem '_waveforms.png']), 'Resolution', 160);
close(fig);

fid = fopen(fullfile(result_dir, [stem '_metrics.json']), 'w');
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true));
fclose(fid);
fprintf('BL%d: pass=%d, blend=%.3f, Vd=%.4f pu, I=%.4f pu, f=%.3f Hz\n', ...
    build_level, metrics.pass, metrics.transition_blend_final, ...
    metrics.voltage_d_mean_pu, metrics.phase_current_rms_pu, metrics.frequency_hz);
end

function out = run_loaded_model(root, model, level, stop_time)
exe = fullfile(root, 'x64', 'Debug', 'Digital_Power_simulink.exe');
header = fileread(fullfile(root, 'sdpe_mgr', ...
    'ctrl_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= level
    error('GFM:BuildLevelMismatch', ...
        'Rebuild the controller with BUILD_LEVEL=%d.', level);
end
info = System.Diagnostics.ProcessStartInfo;
info.FileName = exe;
info.WorkingDirectory = root;
info.UseShellExecute = false;
info.CreateNoWindow = true;
set_param(model, 'SimulationCommand', 'update');
controller = System.Diagnostics.Process.Start(info);
cleanup = onCleanup(@() stop_controller(controller)); %#ok<NASGU>
pause(1.0); % Allow the controller UDP endpoint to bind before Simulink sends the first frame.
out = sim(model, 'StopTime', num2str(stop_time, 17), ...
    'ReturnWorkspaceOutputs', 'on');
end

function model = model_for_level(level)
if ismember(level, [1 2 3])
    model = 'DP_STD_MDL_DCAC_3ph_2level_resload';
elseif level == 4
    model = 'DP_STD_MDL_DCAC_3ph_2level_gridconn';
else
    model = prepare_gfm_pmsg_grid_model();
end
end

function expose_extended_monitors(model)
selectors = find_system(model, 'SearchDepth', 1, 'BlockType', 'BusSelector');
for k = 1:numel(selectors)
    outputs = get_param(selectors{k}, 'OutputSignals');
    if contains(outputs, 'Monitor CH13') && ~contains(outputs, 'Monitor CH15')
        set_param(selectors{k}, 'OutputSignals', ...
            [outputs ',Monitor CH15,Monitor CH16']);
        return;
    end
end
end

function add_monitor_logger(model, channel, variable, index)
pattern = sprintf('Monitor CH%d', channel);
selectors = find_system(model, 'SearchDepth', 1, 'BlockType', 'BusSelector');
source = '';
port = 0;
for k = 1:numel(selectors)
    outputs = strtrim(split(get_param(selectors{k}, 'OutputSignals'), ','));
    match = find(strcmp(outputs, pattern), 1);
    if ~isempty(match), source = selectors{k}; port = match; break; end
end
if isempty(source)
    error('GFM:MonitorChannelMissing', 'Cannot find monitor channel %d.', channel);
end
add_block_logger(model, source, port, variable, index);
end

function add_block_logger(model, source, port, variable, index)
block = sprintf('%s/GFM Validation Logger %02d', model, index);
add_block('simulink/Sinks/To Workspace', block, ...
    'VariableName', variable, 'SaveFormat', 'Timeseries', ...
    'Decimation', '10', ...
    'MaxDataPoints', '2000000', ...
    'Position', [1120 20+index*25 1260 38+index*25]);
s = get_param(source, 'PortHandles');
d = get_param(block, 'PortHandles');
add_line(model, s.Outport(port), d.Inport(1), 'autorouting', 'on');
end

function value = tail_mean(series, start_time)
d = double(series.Data(series.Time >= start_time,:));
d = d(isfinite(d));
value = mean(d, 'all');
end

function value = tail_rms(series, start_time)
d = double(series.Data(series.Time >= start_time,:));
d = d(isfinite(d));
value = sqrt(mean(d.^2, 'all'));
end

function value = window_rms(series, start_time, end_time)
d = double(series.Data(series.Time >= start_time & series.Time <= end_time,:));
d = d(isfinite(d));
value = sqrt(mean(d.^2, 'all'));
end

function value = tail_peak(series, start_time)
d = double(series.Data(series.Time >= start_time,:));
d = d(isfinite(d));
value = max(abs(d), [], 'all');
end

function value = max_step(series)
d = double(series.Data);
d = d(isfinite(d));
value = max(abs(diff(d)), [], 'all');
end

function value = last_value(series)
d = double(series.Data);
value = d(end);
end

function frequency = angle_frequency(series, start_time)
t = double(series.Time);
a = double(series.Data);
mask = t >= start_time;
t = t(mask);
a = a(mask);
a = unwrap(2*pi*a)/(2*pi);
if numel(t) < 2 || t(end) <= t(1)
    frequency = NaN;
else
    fit = polyfit(t - t(1), a - a(1), 1);
    frequency = fit(1);
end
end

function yes = finite_scalar(value)
yes = isnumeric(value) && isscalar(value) && isfinite(value);
end

function stop_controller(controller)
try
    if ~isempty(controller) && ~controller.HasExited
        controller.Kill;
        controller.WaitForExit(2000);
    end
catch
end
end
