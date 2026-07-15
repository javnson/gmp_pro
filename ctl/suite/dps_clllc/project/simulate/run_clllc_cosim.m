function sim_out = run_clllc_cosim(stop_time)
%RUN_CLLLC_COSIM Start the controller process and execute the switching model.
root=fileparts(mfilename('fullpath')); model='GMP_STD_CLLLC_MODEL';
exe=fullfile(root,'x64','Debug','Digital_Power_Suite_CLLLC_SIL_Env.exe');
if ~isfile(exe), error('CLLLC:SILExecutableMissing','Build the x64 Debug SIL executable first.'); end
addpath(genpath(fullfile(root,'..','..','..','..','..','slib','install_path','R2024b')));
load_system(fullfile(root,[model '.slx']));
info=System.Diagnostics.ProcessStartInfo; info.FileName=exe; info.WorkingDirectory=root;
info.UseShellExecute=false; info.CreateNoWindow=true;
controller=System.Diagnostics.Process.Start(info);
cleanup=onCleanup(@()stop_controller(controller)); %#ok<NASGU>
pause(0.3);
if nargin<1 || isempty(stop_time), stop_time=str2double(get_param(model,'StopTime')); end
sim_out=sim(model,'StopTime',num2str(stop_time,17),'ReturnWorkspaceOutputs','on');
close_system(model,0);
end
function stop_controller(p)
try, if ~isempty(p) && ~p.HasExited, p.Kill; p.WaitForExit(2000); end, catch, end
end
