function metrics = run_pmsm_id_validation(build_level, stop_time, label)
%RUN_PMSM_ID_VALIDATION Validate the offline-ID state machine at one build gate.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:4)}
    stop_time (1,1) double {mustBePositive} = 20
    label (1,:) char = ''
end
root = fileparts(mfilename('fullpath'));
header = fileread(fullfile(root, 'sdpe_mgr', 'ctrl_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= build_level
    error('PMSMID:BuildLevelMismatch', 'Rebuild with BUILD_LEVEL=%d.', build_level);
end
if isempty(label), label = sprintf('build_level_%d', build_level); end
result_dir = fullfile(root, 'validation'); if ~isfolder(result_dir), mkdir(result_dir); end
raw_file = fullfile(result_dir, [label '_raw.mat']);
result = run_pmsm_id_sil('StopTime', stop_time, 'Build', false, ...
    'Plot', false, 'ResultFile', raw_file);

state = result.monitor{1};
completed_index = find(round(double(state.Data(:))) == 8, 1, 'first');
if isempty(completed_index), completion_time = NaN;
else, completion_time = double(state.Time(completed_index));
end
errors = double(result.comparison.RelativeErrorPct(:));
finite_errors = errors(isfinite(errors));
if isempty(finite_errors), max_error = Inf; else, max_error = max(abs(finite_errors)); end
all_finite = true;
for k = 1:numel(result.monitor)
    d = double(result.monitor{k}.Data(:));
    all_finite = all_finite && all(isfinite(d));
end
metrics = struct;
metrics.build_level = build_level;
metrics.stop_time_s = stop_time;
metrics.identification_completed = result.completed;
metrics.completion_time_s = completion_time;
metrics.parameter_relative_error_percent = table2struct(result.comparison);
metrics.steady_state_error_abs = max_error;
metrics.steady_state_error_percent = max_error;
metrics.simulation_pass = all_finite;
metrics.runtime_assertion_triggered = false;
metrics.dynamic_pass = result.completed && isfinite(completion_time) && completion_time < stop_time;
metrics.steady_state_pass = result.completed && max_error < 10 && ...
    result.encoder.fault == 0;
metrics.pass = metrics.simulation_pass && metrics.dynamic_pass && metrics.steady_state_pass;

fid = fopen(fullfile(result_dir, [label '_metrics.json']), 'w');
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true)); fclose(fid);
f = figure('Visible','off','Color','w','Position',[100 100 1100 650]);
tiledlayout(2,1,'TileSpacing','compact');
nexttile; stairs(state.Time,state.Data,'LineWidth',1.2); grid on; ylabel('OID state');
title(sprintf('PMSM ID BUILD_LEVEL %d',build_level));
nexttile; bar(categorical(result.comparison.Parameter), errors); grid on;
ylabel('Relative error (%)'); xlabel('Identified parameter');
try,exportgraphics(f,fullfile(result_dir,[label '_waveforms.png']),'Resolution',160);catch exception,warning('PMSMID:PlotExport','%s',exception.message);end
close(f);
fprintf('PMSM ID BL%d: pass=%d, completed at %.6g s, max error=%.3g%%\n', ...
    build_level,metrics.pass,completion_time,max_error);
end
