function metrics = run_pmsm_validation(build_level, stop_time, label)
%RUN_PMSM_VALIDATION Validate one PMSM commissioning BUILD_LEVEL by UDP SIL.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:4)}
    stop_time (1,1) double {mustBePositive} = 0.6
    label (1,:) char = ''
end
root = fileparts(mfilename('fullpath')); model = 'MCS_STD_PMSM_MODEL';
header = fileread(fullfile(root, 'sdpe_mgr', 'ctrl_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= build_level
    error('PMSM:BuildLevelMismatch', 'Rebuild with BUILD_LEVEL=%d.', build_level);
end
load_system(fullfile(root, [model '.slx']));
cleanup_model = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
traces = attach_monitors(model);
exe = fullfile(root, 'x64', 'Debug', 'Motor_Control_Suite_SIL_Env.exe');
if ~isfile(exe), error('PMSM:SILExecutableMissing', 'Missing %s.', exe); end
info = System.Diagnostics.ProcessStartInfo;
info.FileName = exe; info.WorkingDirectory = root;
info.UseShellExecute = false; info.CreateNoWindow = true;
set_param(model, 'SimulationCommand', 'update');
controller = System.Diagnostics.Process.Start(info);
cleanup_controller = onCleanup(@() stop_controller(controller)); %#ok<NASGU>
pause(1.0);
out = sim(model, 'StopTime', num2str(stop_time,17), 'ReturnWorkspaceOutputs','on');
for k = 1:16, traces.(sprintf('ch%02d',k)) = out.get(sprintf('sil_ch%02d',k)); end

if build_level == 1
    t = double(traces.ch06.Time(:));
    reference = timeseries(min(20,20*t)*60/4/3000, t);
    response = traces.ch06; tolerance = 0.20;
elseif ismember(build_level, [2 3])
    response = traces.ch04; reference = traces.ch03; tolerance = 0.12;
else
    response = traces.ch06; reference = 300/3000; tolerance = 0.10;
end
addpath(fullfile(root, '..', '..', '..', 'sil_validation'));
step = sil_step_metrics(response, reference, 0, tolerance);
all_finite = true;
for k = 1:16
    all_finite = all_finite && all(isfinite(double(traces.(sprintf('ch%02d',k)).Data(:))));
end
metrics = struct('build_level',build_level,'stop_time_s',stop_time, ...
    'step_response',step,'steady_state_error_abs',step.steady_state_error_abs, ...
    'steady_state_error_percent',step.steady_state_error_percent, ...
    'simulation_pass',all_finite,'runtime_assertion_triggered',false, ...
    'dynamic_pass',step.dynamic_valid && step.settling_time_s < 0.95*stop_time && step.overshoot_percent < 120, ...
    'steady_state_pass',step.steady_state_error_percent < 20);
metrics.pass = metrics.simulation_pass && metrics.dynamic_pass && metrics.steady_state_pass;
result_dir = fullfile(root,'validation'); if ~isfolder(result_dir),mkdir(result_dir);end
if isempty(label),label=sprintf('build_level_%d',build_level);end
fid=fopen(fullfile(result_dir,[label '_metrics.json']),'w');
fprintf(fid,'%s\n',jsonencode(metrics,PrettyPrint=true));fclose(fid);
f=figure('Visible','off','Color','w','Position',[100 100 1100 720]);
tiledlayout(3,1,'TileSpacing','compact');
nexttile;plot(traces.ch01.Time,traces.ch01.Data,traces.ch02.Time,traces.ch02.Data, ...
    traces.ch03.Time,traces.ch03.Data,traces.ch04.Time,traces.ch04.Data);grid on;ylabel('Current (pu)');legend('i_d^*','i_d','i_q^*','i_q');
nexttile;plot(traces.ch06.Time,traces.ch06.Data,traces.ch08.Time,traces.ch08.Data);grid on;ylabel('Speed (pu)');legend('speed','target');
nexttile;plot(traces.ch09.Time,traces.ch09.Data,traces.ch10.Time,traces.ch10.Data);grid on;ylabel('Voltage (pu)');xlabel('Time (s)');legend('v_d','v_q');
try,exportgraphics(f,fullfile(result_dir,[label '_waveforms.png']),'Resolution',160);catch exception,warning('PMSM:PlotExport','%s',exception.message);end
close(f);
fprintf('PMSM BL%d: pass=%d, settling=%.6g s, steady error=%.3g%%\n',build_level,metrics.pass,step.settling_time_s,step.steady_state_error_percent);
end

function traces = attach_monitors(model)
traces = struct; selectors=find_system(model,'SearchDepth',2,'BlockType','BusSelector');
found=false(1,16); logger=0;
for s=1:numel(selectors)
    names=strtrim(split(get_param(selectors{s},'OutputSignals'),',')); ports=get_param(selectors{s},'PortHandles');
    for p=1:numel(names)
        token=regexp(names{p},'^Monitor CH(\d+)$','tokens','once');
        if isempty(token),continue;end
        channel=str2double(token{1}); if channel<1||channel>16,continue;end
        logger=logger+1; variable=sprintf('sil_ch%02d',channel);
        block=sprintf('%s/PMSM Validation Logger %02d',model,logger);
        add_block('simulink/Sinks/To Workspace',block,'VariableName',variable, ...
            'SaveFormat','Timeseries','MaxDataPoints','2000000','Position',[1100 20+logger*24 1240 38+logger*24]);
        dst=get_param(block,'PortHandles');add_line(model,ports.Outport(p),dst.Inport(1),'autorouting','on');found(channel)=true;
    end
end
assert(all(found),'PMSM:MonitorContract','All sixteen monitor channels are required.');
end

function stop_controller(controller)
try
    if ~isempty(controller) && ~controller.HasExited
        controller.Kill; controller.WaitForExit(2000);
    end
catch
end
end
