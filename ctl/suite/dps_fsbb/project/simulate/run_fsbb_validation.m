function metrics = run_fsbb_validation(build_level, stop_time)
%RUN_FSBB_VALIDATION Run one SIL validation and record controller waveforms.
%   The executable must have been rebuilt with the requested BUILD_LEVEL.
%   Logging blocks are added only to the in-memory model and are not saved.

arguments
    build_level (1,1) double {mustBeMember(build_level, [1 2 3])}
    stop_time (1,1) double {mustBePositive} = 0.8
end

root = fileparts(mfilename('fullpath'));
model = 'MCS_STD_FSBB_MODEL';
exe = fullfile(root, 'x64', 'Debug', 'Digital_Power_Suite_FSBB_SIL_Env.exe');
if ~isfile(exe)
    error('FSBB:SILExecutableMissing', 'Build Debug|x64 first: %s', exe);
end

header = fileread(fullfile(root, 'sdpe_mgr', ...
    'sdpe_dps_fsbb_simulate_settings.h'));
level_token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(level_token) || str2double(level_token{1}) ~= build_level
    error('FSBB:BuildLevelMismatch', ...
        'Generated settings do not select BUILD_LEVEL=%d.', build_level);
end

model_file = fullfile(root, [model '.slx']);
run(fullfile(root, '..', '..', 'sdpe_general', ...
    'sdpe_dps_fsbb_common_settings_matlab_init.m'));
run(fullfile(root, 'sdpe_mgr', ...
    'sdpe_dps_fsbb_simulate_settings_matlab_init.m'));
load_system(model_file);
model_cleanup = onCleanup(@() close_model_without_saving(model));

% Controller monitor contract: Vin, Vout, IL, Iout, formal V command, raw V request.
selectors = {'Bus Selector2', 'Bus Selector2', 'Bus Selector3', ...
    'Bus Selector3', 'Bus Selector4', 'Bus Selector4'};
ports = [1 2 1 2 1 2];
variables = {'mon_vin', 'mon_vout', 'mon_il', 'mon_iout', 'mon_vformal', 'mon_vreq'};
for k = 1:numel(variables)
    add_logger(model, [model '/' selectors{k}], ports(k), variables{k}, k);
end
add_logger(model, [model '/Bus Selector10'], 1, 'pwm_buck', 7);
add_logger(model, [model '/Bus Selector10'], 2, 'pwm_boost', 8);
add_logger(model, [model '/From15'], 1, 'output_enable', 9);
add_logger(model, [model '/GMP STD FSBB Module'], 2, 'raw_adc', 10);
add_logger(model, [model '/Bus Selector5'], 1, 'cia402_state', 11);
add_logger(model, [model '/Bus Selector5'], 2, 'cia402_cmd', 12);

start_info = System.Diagnostics.ProcessStartInfo;
start_info.FileName = exe;
start_info.WorkingDirectory = root;
start_info.UseShellExecute = false;
start_info.CreateNoWindow = true;
controller = System.Diagnostics.Process.Start(start_info);
controller_cleanup = onCleanup(@() stop_controller(controller));
pause(0.25);

out = sim(model, 'StopTime', num2str(stop_time, 17), ...
    'ReturnWorkspaceOutputs', 'on');

vin = out.get('mon_vin');
vout = out.get('mon_vout');
il = out.get('mon_il');
iout = out.get('mon_iout');
vformal = out.get('mon_vformal');
vreq = out.get('mon_vreq');
buck = out.get('pwm_buck');
boost = out.get('pwm_boost');
enable = out.get('output_enable');
adc = out.get('raw_adc');
state = out.get('cia402_state');
command = out.get('cia402_cmd');

settle_start = 0.8 * stop_time;
metrics = struct( ...
    'build_level', build_level, ...
    'stop_time_s', stop_time, ...
    'vin_final_v', tail_mean(vin, settle_start), ...
    'vout_final_v', tail_mean(vout, settle_start), ...
    'vout_peak_v', max(vout.Data), ...
    'il_final_a', tail_mean(il, settle_start), ...
    'iout_final_a', tail_mean(iout, settle_start), ...
    'vformal_final_v', tail_mean(vformal, settle_start), ...
    'vrequest_final_v', tail_mean(vreq, settle_start), ...
    'enable_final', enable.Data(end), ...
    'cia402_state_final', state.Data(end), ...
    'cia402_command_final', command.Data(end), ...
    'adc_final_codes', reshape(double(adc.Data(end, :)), 1, []));

result_dir = fullfile(root, 'validation');
if ~isfolder(result_dir)
    mkdir(result_dir);
end
stem = sprintf('build_level_%d', build_level);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 850]);
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(vin.Time, vin.Data, '-', vout.Time, vout.Data, '-', ...
    vformal.Time, vformal.Data, '--', vreq.Time, vreq.Data, ':', 'LineWidth', 1.1);
grid on; ylabel('Voltage (V)');
legend('V_{in}', 'V_{out}', 'V_{formal}', 'V_{request}', 'Location', 'best');
title(sprintf('FSBB UDP SIL validation - BUILD_LEVEL=%d', build_level), 'Interpreter', 'none');

nexttile;
plot(il.Time, il.Data, '-', iout.Time, iout.Data, '-', 'LineWidth', 1.1);
grid on; ylabel('Current (A)'); legend('I_L', 'I_{out}', 'Location', 'best');

nexttile;
cmp_max = CTRL_PWM_CMP_MAX;
plot(buck.Time, double(buck.Data) / cmp_max, '-', ...
    boost.Time, double(boost.Data) / cmp_max, '-', ...
    enable.Time, double(enable.Data), '--', 'LineWidth', 1.0);
grid on; ylabel('Command'); xlabel('Time (s)'); ylim([-0.05 1.05]);
legend('Buck effective duty', 'Boost effective duty', 'Enable', 'Location', 'best');
exportgraphics(fig, fullfile(result_dir, [stem '_waveforms.png']), 'Resolution', 160);
close(fig);

fid = fopen(fullfile(result_dir, [stem '_metrics.json']), 'w');
file_cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true));

fprintf(['BUILD_LEVEL=%d: enable=%g, Vin=%.3f V, Vout=%.3f V, ' ...
    'IL=%.3f A, Iout=%.3f A, Vreq=%.3f V\n'], ...
    build_level, metrics.enable_final, metrics.vin_final_v, ...
    metrics.vout_final_v, metrics.il_final_a, metrics.iout_final_a, ...
    metrics.vrequest_final_v);
end

function add_logger(model, source, source_port, variable, index)
block = sprintf('%s/Validation Logger %02d', model, index);
add_block('simulink/Sinks/To Workspace', block, ...
    'VariableName', variable, 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', '1000000', 'Position', [1100 40 + index * 30 1240 60 + index * 30]);
src = get_param(source, 'PortHandles');
dst = get_param(block, 'PortHandles');
add_line(model, src.Outport(source_port), dst.Inport(1), 'autorouting', 'on');
end

function value = tail_mean(series, start_time)
data = double(series.Data(series.Time >= start_time, :));
value = mean(data, 'all');
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

function close_model_without_saving(model)
try
    close_system(model, 0);
catch
end
end
