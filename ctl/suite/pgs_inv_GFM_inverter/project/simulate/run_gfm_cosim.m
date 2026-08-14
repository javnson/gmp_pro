function sim_out = run_gfm_cosim(build_level, stop_time)
%RUN_GFM_COSIM Launch the controller executable and matching three-phase plant.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:5)}
    stop_time (1,1) double {mustBePositive} = 2.0
end

root = fileparts(mfilename('fullpath'));
model = model_for_level(build_level);
exe = fullfile(root, 'x64', 'Debug', 'Digital_Power_simulink.exe');
assert(isfile(exe), 'GFM:SILExecutableMissing', ...
    'Build Debug|x64 before simulation: %s', exe);
assert_build_level(root, build_level);

load_system(fullfile(root, [model '.slx']));
model_cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
set_param(model, 'SimulationCommand', 'update');
controller = start_controller(exe, root);
controller_cleanup = onCleanup(@() stop_controller(controller)); %#ok<NASGU>
pause(1.0); % Allow the controller UDP endpoint to bind before Simulink sends the first frame.
sim_out = sim(model, 'StopTime', num2str(stop_time, 17), ...
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

function assert_build_level(root, level)
header = fileread(fullfile(root, 'sdpe_mgr', ...
    'ctrl_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', ...
    'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= level
    error('GFM:BuildLevelMismatch', ...
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
