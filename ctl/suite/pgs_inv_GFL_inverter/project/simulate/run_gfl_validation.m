function metrics = run_gfl_validation(build_level, stop_time, label)
%RUN_GFL_VALIDATION Run one UDP SIL case and save objective controller metrics.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:6)}
    stop_time (1,1) double {mustBePositive} = 1.0
    label (1,:) char = ''
end

root = fileparts(mfilename('fullpath'));
model = model_for_level(build_level);
load_system(fullfile(root, [model '.slx']));
cleanup_model = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
expose_extended_monitors(model);

add_monitor_logger(model, 1, 'ia_pu', 1);
add_monitor_logger(model, 2, 'ib_pu', 2);
add_monitor_logger(model, 3, 'i0_pu', 3);
add_monitor_logger(model, 5, 'v_alpha_pu', 4);
add_monitor_logger(model, 6, 'v_beta_pu', 5);
add_monitor_logger(model, 7, 'pll_error_pu', 6);
add_monitor_logger(model, 11, 'id_pu', 7);
add_monitor_logger(model, 12, 'iq_pu', 8);
add_monitor_logger(model, 13, 'vd_pu', 9);
add_monitor_logger(model, 14, 'vq_pu', 10);
add_monitor_logger(model, 15, 'voltage_id_ref_pu', 11);
add_monitor_logger(model, 16, 'zero_voltage_pu', 12);
add_block_logger(model, [model '/From11'], 1, 'output_enable', 13);

out = run_loaded_model(root, model, build_level, stop_time);
names = {'ia_pu','ib_pu','i0_pu','v_alpha_pu','v_beta_pu', ...
    'pll_error_pu','id_pu','iq_pu','vd_pu','vq_pu', ...
    'voltage_id_ref_pu','zero_voltage_pu','output_enable'};
signals = struct;
for k = 1:numel(names)
    signals.(names{k}) = out.get(names{k});
end

tail_time = 0.65 * stop_time;
metrics = struct;
metrics.build_level = build_level;
metrics.model = model;
metrics.stop_time_s = stop_time;
metrics.output_enable_final = last_value(signals.output_enable);
metrics.phase_current_rms_pu = tail_rms(signals.ia_pu, tail_time);
metrics.zero_current_rms_pu = tail_rms(signals.i0_pu, tail_time);
metrics.voltage_alpha_rms_pu = tail_rms(signals.v_alpha_pu, tail_time);
metrics.voltage_beta_rms_pu = tail_rms(signals.v_beta_pu, tail_time);
metrics.voltage_d_mean_pu = tail_mean(signals.vd_pu, tail_time);
metrics.voltage_q_rms_pu = tail_rms(signals.vq_pu, tail_time);
metrics.current_d_mean_pu = tail_mean(signals.id_pu, tail_time);
metrics.current_q_mean_pu = tail_mean(signals.iq_pu, tail_time);
metrics.pll_error_rms_pu = tail_rms(signals.pll_error_pu, tail_time);
metrics.voltage_loop_current_ref_peak_pu = ...
    tail_peak(signals.voltage_id_ref_pu, tail_time);
metrics.zero_voltage_rms_pu = tail_rms(signals.zero_voltage_pu, tail_time);
metrics.command_d_mean_pu = tail_mean(signals.voltage_id_ref_pu, tail_time);
metrics.command_q_mean_pu = tail_mean(signals.zero_voltage_pu, tail_time);
metrics.finite = all(structfun(@finite_scalar, ...
    rmfield(metrics, {'model'})));

if build_level == 1
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.voltage_alpha_rms_pu > 0.20;
elseif build_level == 2
    metrics.current_tracking_error_pu = hypot( ...
        metrics.current_d_mean_pu - 0.10, metrics.current_q_mean_pu - 0.10);
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.current_tracking_error_pu < 0.05;
elseif build_level == 3
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        abs(metrics.current_d_mean_pu - 0.10) < 0.05 && ...
        abs(metrics.current_q_mean_pu) < 0.05;
elseif build_level == 4
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        hypot(metrics.current_d_mean_pu, metrics.current_q_mean_pu) > 0.10;
elseif build_level == 5
    metrics.droop_active_power_ref_pu = metrics.command_d_mean_pu;
    metrics.droop_reactive_power_ref_pu = metrics.command_q_mean_pu;
    metrics.droop_reference_change_pu = hypot( ...
        metrics.droop_active_power_ref_pu - 0.10, ...
        metrics.droop_reactive_power_ref_pu);
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        abs(metrics.droop_active_power_ref_pu) < 0.805 && ...
        abs(metrics.droop_reactive_power_ref_pu) < 0.805 && ...
        metrics.droop_reference_change_pu > 0.02 && ...
        hypot(metrics.current_d_mean_pu, metrics.current_q_mean_pu) > 0.03;
else
    metrics.voltage_reference_pu = 0.50;
    metrics.voltage_tracking_error_pu = ...
        abs(metrics.voltage_d_mean_pu - metrics.voltage_reference_pu);
    metrics.pass = metrics.finite && metrics.output_enable_final > 0.5 && ...
        metrics.voltage_tracking_error_pu < 0.08 && ...
        metrics.voltage_loop_current_ref_peak_pu <= 0.805;
end

result_dir = fullfile(root, 'validation');
if ~isfolder(result_dir), mkdir(result_dir); end
if isempty(label), label = sprintf('build_level_%d', build_level); end
stem = matlab.lang.makeValidName(label);

fig = figure('Visible','off','Color','w','Position',[100 100 1200 850]);
tiledlayout(fig, 3, 1, 'TileSpacing','compact','Padding','compact');
nexttile;
plot(signals.v_alpha_pu.Time, signals.v_alpha_pu.Data, ...
    signals.v_beta_pu.Time, signals.v_beta_pu.Data, 'LineWidth', 1.0);
grid on; ylabel('Voltage (pu)'); legend('v_\alpha','v_\beta');
title(strrep(label, '_', ' '));
nexttile;
plot(signals.ia_pu.Time, signals.ia_pu.Data, ...
    signals.ib_pu.Time, signals.ib_pu.Data, ...
    signals.i0_pu.Time, signals.i0_pu.Data, 'LineWidth', 1.0);
grid on; ylabel('Current (pu)'); legend('i_a','i_b','i_0');
nexttile;
plot(signals.vd_pu.Time, signals.vd_pu.Data, ...
    signals.voltage_id_ref_pu.Time, signals.voltage_id_ref_pu.Data, ...
    signals.zero_voltage_pu.Time, signals.zero_voltage_pu.Data, 'LineWidth', 1.0);
grid on; ylabel('Control (pu)'); xlabel('Time (s)');
if build_level == 5
    legend('v_d','P^* from PQ droop','Q^* from PQ droop');
else
    legend('v_d','i_d^* from voltage loop','v_0^*');
end
exportgraphics(fig, fullfile(result_dir, [stem '_waveforms.png']), ...
    'Resolution', 160);
close(fig);

fid = fopen(fullfile(result_dir, [stem '_metrics.json']), 'w');
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true));
fclose(fid);
fprintf('BL%d: pass=%d, enable=%.0f, Vd=%.4f pu, I=%.4f pu, I0=%.5f pu\n', ...
    build_level, metrics.pass, metrics.output_enable_final, ...
    metrics.voltage_d_mean_pu, metrics.phase_current_rms_pu, ...
    metrics.zero_current_rms_pu);
end

function out = run_loaded_model(root, model, level, stop_time)
exe = fullfile(root, 'x64', 'Debug', 'Digital_Power_simulink.exe');
header = fileread(fullfile(root, 'sdpe_mgr', ...
    'ctrl_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', ...
    'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= level
    error('GFL:BuildLevelMismatch', ...
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
if ismember(level, [1 2 6])
    model = 'DP_STD_MDL_DCAC_3ph_2level_resload';
else
    model = 'DP_STD_MDL_DCAC_3ph_2level_gridconn';
end
end

function expose_extended_monitors(model)
selectors = find_system(model, 'SearchDepth', 1, ...
    'BlockType', 'BusSelector');
for k = 1:numel(selectors)
    outputs = get_param(selectors{k}, 'OutputSignals');
    if contains(outputs, 'Monitor CH13') && ...
            ~contains(outputs, 'Monitor CH15')
        set_param(selectors{k}, 'OutputSignals', ...
            [outputs ',Monitor CH15,Monitor CH16']);
        return;
    end
end
end

function add_monitor_logger(model, channel, variable, index)
pattern = sprintf('Monitor CH%d', channel);
selectors = find_system(model, 'SearchDepth', 1, ...
    'BlockType', 'BusSelector');
source = '';
port = 0;
for k = 1:numel(selectors)
    outputs = strtrim(split(get_param(selectors{k}, 'OutputSignals'), ','));
    match = find(strcmp(outputs, pattern), 1);
    if ~isempty(match)
        source = selectors{k};
        port = match;
        break;
    end
end
if isempty(source)
    error('GFL:MonitorChannelMissing', ...
        'Cannot find monitor channel %d.', channel);
end
add_block_logger(model, source, port, variable, index);
end

function add_block_logger(model, source, port, variable, index)
block = sprintf('%s/GFL Validation Logger %02d', model, index);
add_block('simulink/Sinks/To Workspace', block, ...
    'VariableName', variable, 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', '2000000', ...
    'Position', [1120 20+index*25 1260 38+index*25]);
s = get_param(source, 'PortHandles');
d = get_param(block, 'PortHandles');
add_line(model, s.Outport(port), d.Inport(1), 'autorouting', 'on');
end

function value = tail_mean(series, start_time)
d = double(series.Data(series.Time >= start_time,:));
value = mean(d, 'all');
end

function value = tail_rms(series, start_time)
d = double(series.Data(series.Time >= start_time,:));
value = sqrt(mean(d.^2, 'all'));
end

function value = tail_peak(series, start_time)
d = double(series.Data(series.Time >= start_time,:));
value = max(abs(d), [], 'all');
end

function value = last_value(series)
d = double(series.Data);
value = d(end);
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
