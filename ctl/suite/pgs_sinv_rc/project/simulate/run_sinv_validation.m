function metrics = run_sinv_validation(build_level, stop_time, label)
%RUN_SINV_VALIDATION Run one UDP SIL case, save metrics and controller waveforms.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:5)}
    stop_time (1,1) double {mustBePositive} = 2.0
    label (1,:) char = ''
end
root = fileparts(mfilename('fullpath'));
if build_level <= 2
    model = 'PGS_STD_SINV_MODEL_RLOAD';
elseif build_level <= 4
    model = 'PGS_STD_SINV_MODEL_Grid';
else
    model = 'PGS_STD_SINV_MODEL_Rectifier';
end
exe = fullfile(root, 'x64', 'Debug', 'Motor_Control_Suite_SIL_Env.exe');
header = fileread(fullfile(root, 'sdpe_mgr', 'sdpe_pgs_sinv_rc_simulate_settings.h'));
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= build_level
    error('SINV:BuildLevelMismatch', 'Rebuild the executable with BUILD_LEVEL=%d.', build_level);
end

load_system(fullfile(root, [model '.slx']));
cleanup_model = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
% Keep the validation plant tied to the latest generated SDPE values even
% when the model is open (and therefore write-locked) in another MATLAB.
if build_level == 5
    set_param([model '/Load1'], 'BranchType', 'R', ...
        'Resistance', 'SINV_RECTIFIER_RLOAD_OHM');
end
selectors = {'Bus Selector2','Bus Selector2','Bus Selector3','Bus Selector3', ...
    'Bus Selector4','Bus Selector4','Bus Selector5','Bus Selector5', ...
    'Bus Selector6','Bus Selector6','Bus Selector7','Bus Selector7', ...
    'Bus Selector8','Bus Selector8'};
variables = {'vac','iac','vbus','iref','modulation','pll_hz','p_pu','q_pu', ...
    'current_error','qpr_output','fdrc_output','fdrc_enabled','cia402_state','cia402_cmd', ...
    'active_errors','diverge_fault_value'};
selectors = [selectors, {'Bus Selector9','Bus Selector9'}];
for k = 1:numel(variables)
    add_logger(model, [model '/' selectors{k}], 1 + mod(k-1, 2), variables{k}, k);
end
add_logger(model, [model '/Bus Selector'], 1, 'pwm_l', 20);
add_logger(model, [model '/Bus Selector'], 2, 'pwm_n', 21);
add_logger(model, [model '/From'], 1, 'output_enable', 22);

info = System.Diagnostics.ProcessStartInfo;
info.FileName = exe;
info.WorkingDirectory = root;
info.UseShellExecute = false;
info.CreateNoWindow = true;
controller = System.Diagnostics.Process.Start(info);
cleanup_controller = onCleanup(@() stop_controller(controller)); %#ok<NASGU>
pause(0.25);
out = sim(model, 'StopTime', num2str(stop_time, 17), 'ReturnWorkspaceOutputs', 'on');

vac = out.get('vac'); iac = out.get('iac'); vbus = out.get('vbus');
iref = out.get('iref'); modulation = out.get('modulation'); pll_hz = out.get('pll_hz');
p_pu = out.get('p_pu'); q_pu = out.get('q_pu'); error = out.get('current_error');
fdrc = out.get('fdrc_output'); fdrc_enabled = out.get('fdrc_enabled');
enable = out.get('output_enable'); state = out.get('cia402_state');
active_errors = out.get('active_errors'); diverge_fault = out.get('diverge_fault_value');
tail = 0.6 * stop_time;

metrics = struct('build_level', build_level, 'model', model, ...
    'stop_time_s', stop_time, 'output_enable_final', double(enable.Data(end)), ...
    'cia402_state_final', double(state.Data(end)), ...
    'active_errors_final', double(active_errors.Data(end)), ...
    'diverge_fault_value', double(diverge_fault.Data(end)), ...
    'vac_rms_v', tail_rms(vac, tail), 'iac_rms_a', tail_rms(iac, tail), ...
    'vbus_mean_v', tail_mean(vbus, tail), 'iref_rms_a', tail_rms(iref, tail), ...
    'current_error_rms_pu', tail_rms(error, tail), ...
    'active_power_pu', tail_mean(p_pu, tail), 'reactive_power_pu', tail_mean(q_pu, tail), ...
    'pll_frequency_hz', tail_mean(pll_hz, tail), ...
    'fdrc_enabled_final', double(fdrc_enabled.Data(end)), ...
    'fdrc_output_rms_pu', tail_rms(fdrc, tail), ...
    'iac_thd_percent', signal_thd(iac, tail, 50.0));

result_dir = fullfile(root, 'validation');
if ~isfolder(result_dir), mkdir(result_dir); end
if isempty(label), label = sprintf('build_level_%d', build_level); end
stem = matlab.lang.makeValidName(label);
fig = figure('Visible','off','Color','w','Position',[100 100 1200 900]);
tiledlayout(fig, 4, 1, 'TileSpacing','compact','Padding','compact');
nexttile; plot(vac.Time,vac.Data, iac.Time,iac.Data, 'LineWidth',1.0); grid on;
ylabel('V / A'); legend('v_{ac}','i_{ac}'); title(strrep(label,'_',' '));
nexttile; plot(iref.Time,iref.Data, iac.Time,iac.Data, 'LineWidth',1.0); grid on;
ylabel('Current (A)'); legend('i^*','i');
nexttile; plot(vbus.Time,vbus.Data, p_pu.Time,p_pu.Data, q_pu.Time,q_pu.Data, 'LineWidth',1.0); grid on;
ylabel('Bus / power'); legend('V_{dc}','P (pu)','Q (pu)');
nexttile; plot(modulation.Time,modulation.Data, fdrc.Time,fdrc.Data, ...
    fdrc_enabled.Time,fdrc_enabled.Data,'LineWidth',1.0); grid on;
ylabel('Controller'); xlabel('Time (s)'); legend('modulation','FDRC output','FDRC enabled');
exportgraphics(fig, fullfile(result_dir, [stem '_waveforms.png']), 'Resolution', 160);
close(fig);
fid = fopen(fullfile(result_dir, [stem '_metrics.json']), 'w');
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true)); fclose(fid);
fprintf('BL%d %s: enable=%g, Vac=%.3f Vrms, Iac=%.3f Arms, Vdc=%.3f V, THD=%.2f%%\n', ...
    build_level, model, metrics.output_enable_final, metrics.vac_rms_v, ...
    metrics.iac_rms_a, metrics.vbus_mean_v, metrics.iac_thd_percent);
end

function add_logger(model, source, port, variable, index)
block = sprintf('%s/SINV Validation Logger %02d', model, index);
add_block('simulink/Sinks/To Workspace', block, 'VariableName', variable, ...
    'SaveFormat','Timeseries','MaxDataPoints','2000000', ...
    'Position',[1100 20+index*24 1240 38+index*24]);
s = get_param(source,'PortHandles'); d = get_param(block,'PortHandles');
add_line(model,s.Outport(port),d.Inport(1),'autorouting','on');
end

function value = tail_mean(series, start_time)
d = double(series.Data(series.Time >= start_time,:)); value = mean(d,'all');
end

function value = tail_rms(series, start_time)
d = double(series.Data(series.Time >= start_time,:)); value = sqrt(mean(d.^2,'all'));
end

function thd_percent = signal_thd(series, start_time, fundamental)
t = double(series.Time); x = double(series.Data(:)); keep = t >= start_time;
t = t(keep); x = x(keep); fs = 20000; tu = (t(1):1/fs:t(end))';
xu = interp1(t,x,tu,'linear','extrap'); xu = xu - mean(xu);
ncycle = floor((tu(end)-tu(1))*fundamental); n = floor(ncycle*fs/fundamental);
if n < fs/fundamental, thd_percent = NaN; return; end
xu = xu(end-n+1:end); spectrum = abs(fft(xu))/n*2;
bin = round(fundamental*n/fs)+1; fundamental_amp = spectrum(bin);
harmonic_sq = 0;
for h = 2:min(40,floor((fs/2)/fundamental))
    harmonic_sq = harmonic_sq + spectrum(round(h*fundamental*n/fs)+1)^2;
end
thd_percent = 100*sqrt(harmonic_sq)/max(fundamental_amp,eps);
end

function stop_controller(controller)
try
    if ~isempty(controller) && ~controller.HasExited
        controller.Kill; controller.WaitForExit(2000);
    end
catch
end
end
