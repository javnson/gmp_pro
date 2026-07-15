function sim_out = run_sinv_cosim(build_level, stop_time)
%RUN_SINV_COSIM Launch the controller executable and its matching SINV plant.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:5)}
    stop_time (1,1) double {mustBePositive} = NaN
end
root = fileparts(mfilename('fullpath'));
model = model_for_level(build_level);
exe = fullfile(root, 'x64', 'Debug', 'Motor_Control_Suite_SIL_Env.exe');
if ~isfile(exe)
    error('SINV:SILExecutableMissing', 'Build Debug|x64 first: %s', exe);
end
assert_level(root, build_level);
load_system(fullfile(root, [model '.slx']));
model_cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
controller = start_controller(exe, root);
controller_cleanup = onCleanup(@() stop_controller(controller)); %#ok<NASGU>
pause(0.25);
if isnan(stop_time)
    sim_out = sim(model, 'ReturnWorkspaceOutputs', 'on');
else
    sim_out = sim(model, 'StopTime', num2str(stop_time, 17), ...
        'ReturnWorkspaceOutputs', 'on');
end
end

function model = model_for_level(level)
if level <= 2
    model = 'PGS_STD_SINV_MODEL_RLOAD';
elseif level <= 4
    model = 'PGS_STD_SINV_MODEL_Grid';
else
    model = 'PGS_STD_SINV_MODEL_Rectifier';
end
end

function assert_level(root, level)
header = fileread(fullfile(root, 'sdpe_mgr', 'sdpe_pgs_sinv_rc_simulate_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= level
    error('SINV:BuildLevelMismatch', ...
        'Generated settings/executable do not select BUILD_LEVEL=%d.', level);
end
end

function process = start_controller(exe, root)
info = System.Diagnostics.ProcessStartInfo;
info.FileName = exe;
info.WorkingDirectory = root;
info.UseShellExecute = false;
info.CreateNoWindow = true;
process = System.Diagnostics.Process.Start(info);
end

function stop_controller(process)
try
    if ~isempty(process) && ~process.HasExited
        process.Kill;
        process.WaitForExit(2000);
    end
catch
end
end
