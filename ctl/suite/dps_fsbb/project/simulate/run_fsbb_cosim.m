function sim_out = run_fsbb_cosim(stop_time)
%RUN_FSBB_COSIM Launch the FSBB controller and run the UDP SIL model.
%   SIM_OUT = RUN_FSBB_COSIM() runs the committed model for its configured
%   stop time. RUN_FSBB_COSIM(T) overrides the stop time in seconds.

root = fileparts(mfilename('fullpath'));
model = 'MCS_STD_FSBB_MODEL';
exe = fullfile(root, 'x64', 'Debug', 'Digital_Power_Suite_FSBB_SIL_Env.exe');
if ~isfile(exe)
    error('FSBB:SILExecutableMissing', ...
        'Build Debug|x64 first; controller executable not found: %s', exe);
end

model_file = fullfile(root, [model '.slx']);
run(fullfile(root, '..', '..', 'sdpe_general', ...
    'sdpe_dps_fsbb_common_settings_matlab_init.m'));
run(fullfile(root, 'sdpe_mgr', ...
    'sdpe_dps_fsbb_simulate_settings_matlab_init.m'));
load_system(model_file);

start_info = System.Diagnostics.ProcessStartInfo;
start_info.FileName = exe;
start_info.WorkingDirectory = root;
start_info.UseShellExecute = false;
start_info.CreateNoWindow = true;
controller = System.Diagnostics.Process.Start(start_info);
cleanup = onCleanup(@() stop_controller(controller));

pause(0.25); % Let the UDP endpoint bind before mdlStart sends its handshake.
if nargin < 1 || isempty(stop_time)
    sim_out = sim(model, 'ReturnWorkspaceOutputs', 'on');
else
    validateattributes(stop_time, {'numeric'}, {'scalar', 'positive', 'finite'});
    sim_out = sim(model, 'StopTime', num2str(stop_time, 17), ...
        'ReturnWorkspaceOutputs', 'on');
end
close_system(model, 0);
end

function stop_controller(controller)
if isempty(controller)
    return;
end
try
    if ~controller.HasExited
        controller.Kill;
        controller.WaitForExit(2000);
    end
catch
    % Cleanup must not hide the simulation result or its original error.
end
end
