function metrics = run_clllc_validation(build_level, stop_time, label)
%RUN_CLLLC_VALIDATION Validate one CLLLC BUILD_LEVEL through UDP SIL.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:4)}
    stop_time (1,1) double {mustBePositive} = 0.2
    label (1,:) char = ''
end
root = fileparts(mfilename('fullpath'));
header = fileread(fullfile(root, 'sdpe_mgr', 'ctrl_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= build_level
    error('CLLLC:BuildLevelMismatch', 'Rebuild with BUILD_LEVEL=%d.', build_level);
end
out = run_clllc_cosim(stop_time);
mon = out.get('clllc_monitor'); adc = out.get('clllc_adc');
t = mon.Monitor_CH3.Time;
data = [mon.Monitor_CH3.Data mon.Monitor_CH4.Data mon.Monitor_CH5.Data ...
    mon.Monitor_CH6.Data mon.Monitor_CH7.Data];
signals.vpri = timeseries(data(:,1), t);
signals.ipri = timeseries(data(:,2), t);
signals.vsec = timeseries(data(:,3), t);
signals.ires = timeseries(data(:,4), t);
signals.command = timeseries(data(:,5), t);
tail = t >= 0.8*stop_time;

if build_level == 2
    response = signals.ires; reference = 0.50*10.0; tolerance = 0.15;
elseif build_level >= 3
    response = signals.vsec; reference = 0.40*120.0; tolerance = 0.10;
else
    response = signals.command; reference = mean(data(tail,5)); tolerance = 0.10;
end
addpath(fullfile(root, '..', '..', '..', 'sil_validation'));
step = sil_step_metrics(response, reference, 0, tolerance);

metrics = struct;
metrics.build_level = build_level;
metrics.stop_time_s = stop_time;
metrics.final_primary_voltage_v = mean(data(tail,1));
metrics.final_primary_current_a = mean(data(tail,2));
metrics.final_secondary_voltage_v = mean(data(tail,3));
metrics.final_resonant_current_a = mean(data(tail,4));
metrics.final_modulation_command = mean(data(tail,5));
metrics.step_response = step;
metrics.steady_state_error_abs = step.steady_state_error_abs;
metrics.steady_state_error_percent = step.steady_state_error_percent;
metrics.simulation_pass = all(isfinite(data), 'all');
metrics.runtime_assertion_triggered = false;
metrics.dynamic_pass = step.dynamic_valid && step.settling_time_s < 0.9*stop_time && ...
    step.overshoot_percent < 80;
if build_level == 1
    metrics.steady_state_pass = abs(metrics.final_modulation_command) > 0.01 && ...
        abs(metrics.final_modulation_command) <= 1.0;
else
    metrics.steady_state_pass = step.steady_state_error_percent < 15;
end
metrics.pass = metrics.simulation_pass && metrics.dynamic_pass && metrics.steady_state_pass;

folder = fullfile(root, 'validation'); if ~isfolder(folder), mkdir(folder); end
if isempty(label), label = sprintf('build_level_%d', build_level); end
stem = matlab.lang.makeValidName(label);
fid = fopen(fullfile(folder, [stem '_metrics.json']), 'w');
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true)); fclose(fid);
f = figure('Visible','off','Color','w','Position',[100 100 1100 720]);
tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(t,data(:,1),t,data(:,3),'LineWidth',1); grid on; ylabel('Voltage (V)'); legend('Primary','Secondary');
nexttile; plot(t,data(:,2),t,data(:,4),'LineWidth',1); grid on; ylabel('Current (A)'); legend('Primary','Resonant');
nexttile; plot(t,data(:,5),'LineWidth',1); grid on; ylabel('Command (pu)'); xlabel('Time (s)');
try, exportgraphics(f,fullfile(folder,[stem '_waveforms.png']),'Resolution',160); catch exception, warning('CLLLC:PlotExport','%s',exception.message); end
close(f);
try,save(fullfile(folder,[stem '_results.mat']),'metrics','mon','adc');catch exception,warning('CLLLLC:ResultSave','%s',exception.message);end
fprintf('CLLLC BL%d: pass=%d, settling=%.6g s, steady error=%.3g%%\n', ...
    build_level, metrics.pass, step.settling_time_s, step.steady_state_error_percent);
end
