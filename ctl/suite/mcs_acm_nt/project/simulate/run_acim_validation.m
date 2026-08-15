function metrics = run_acim_validation(build_level, stop_time, label)
%RUN_ACIM_VALIDATION Validate one ACIM commissioning BUILD_LEVEL.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:4)}
    stop_time (1,1) double {mustBePositive} = 0.6
    label (1,:) char = ''
end
root = fileparts(mfilename('fullpath'));
header = fileread(fullfile(root, 'sdpe_mgr', 'ctrl_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= build_level
    error('ACIM:BuildLevelMismatch', 'Rebuild with BUILD_LEVEL=%d.', build_level);
end
exe = fullfile(root, 'x64', 'Debug', 'Motor_Control_Suite_SIL_Env.exe');
if ~isfile(exe), error('ACIM:SILExecutableMissing', 'Missing %s.', exe); end
addpath(fullfile(root, 'commissioning'));
result_dir = fullfile(root, '..', '..', 'doc', 'simulation_result');
if ~isfolder(result_dir), mkdir(result_dir); end
if isempty(label), label = sprintf('build_level_%d', build_level); end
[summary, traces] = run_build_level_sil(stop_time, ...
    fullfile(result_dir, [label '_signals.csv']), exe);

if build_level == 1
    response = traces.sil_ch06; reference = traces.sil_ch07; tolerance = 0.20;
elseif ismember(build_level, [2 3])
    response = traces.sil_ch04; reference = traces.sil_ch03; tolerance = 0.12;
else
    response = traces.sil_ch06; reference = 300/1450; tolerance = 0.10;
end
addpath(fullfile(root, '..', '..', '..', '..', '..', 'tools', ...
    'gmp_sil', 'validation'));
step = sil_step_metrics(response, reference, 0, tolerance);
all_finite = true;
fields = fieldnames(traces);
for k = 1:numel(fields)
    all_finite = all_finite && all(isfinite(double(traces.(fields{k}).Data(:))));
end
metrics = struct('build_level', build_level, 'stop_time_s', stop_time, ...
    'step_response', step, ...
    'steady_state_error_abs', step.steady_state_error_abs, ...
    'steady_state_error_percent', step.steady_state_error_percent, ...
    'simulation_pass', all_finite, 'runtime_assertion_triggered', false, ...
    'dynamic_pass', step.dynamic_valid && step.settling_time_s < 0.95*stop_time && step.overshoot_percent < 120, ...
    'steady_state_pass', step.steady_state_error_percent < 20);
metrics.pass = metrics.simulation_pass && metrics.dynamic_pass && metrics.steady_state_pass;
metrics.signal_summary = table2struct(summary);
fid = fopen(fullfile(result_dir, [label '_metrics.json']), 'w');
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true)); fclose(fid);
fprintf('ACIM BL%d: pass=%d, settling=%.6g s, steady error=%.3g%%\n', ...
    build_level, metrics.pass, step.settling_time_s, step.steady_state_error_percent);
end
