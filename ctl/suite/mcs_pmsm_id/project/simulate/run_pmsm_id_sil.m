function result = run_pmsm_id_sil(varargin)
%RUN_PMSM_ID_SIL Build, run and evaluate the PMSM identification SIL model.
%
% result = run_pmsm_id_sil()
% result = run_pmsm_id_sil('StopTime', 40, 'Build', false, ...
%     'Plot', false, 'ResultFile', 'pmsm_id_sil_result.mat')
%
% The function starts the native Windows controller, runs the existing
% Simulink plant, logs all 16 SIL monitor channels without changing the SLX
% file, and compares the identified electrical and sensored mechanical
% parameters with values configured in the model mask when available.

parser = inputParser;
parser.addParameter('StopTime', 40, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter('Build', true, @(x) islogical(x) && isscalar(x));
parser.addParameter('Plot', true, @(x) islogical(x) && isscalar(x));
parser.addParameter('SimulationMode', 'normal', ...
    @(x) any(strcmpi(char(x), {'normal', 'accelerator'})));
parser.addParameter('PlantSampleTime', 1e-5, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter('ResultFile', '', @(x) ischar(x) || isstring(x));
parser.parse(varargin{:});
options = parser.Results;

project_dir = fileparts(mfilename('fullpath'));
repo_root = getenv('GMP_PRO_LOCATION');
if isempty(repo_root) || ~isfile(fullfile(repo_root, 'gmp_core.h'))
    error('GMP:GmpLocation', ...
        'GMP_PRO_LOCATION must point to the GMP repository before running SIL.');
end

model_path = fullfile(project_dir, 'MCS_STD_PMSM_MODEL.slx');
solution_path = fullfile(project_dir, 'GMP_Motor_Control_simulink.sln');
exe_path = fullfile(project_dir, 'x64', 'Debug', ...
    'Motor_Control_Suite_SIL_Env.exe');
trace_path = fullfile(project_dir, 'pmsm_id_sil_state.log');

if options.Build
    build_sil_controller(solution_path);
end
if ~isfile(exe_path)
    error('GMP:MissingSilExecutable', ...
        'SIL executable does not exist: %s', exe_path);
end

old_dir = cd(project_dir);
dir_cleanup = onCleanup(@() cd(old_dir));
if isfile(trace_path)
    delete(trace_path);
end

load_system(model_path);
[~, model_name] = fileparts(model_path);
model_cleanup = onCleanup(@() close_system(model_name, 0));

monitor_names = configure_monitor_logging(model_name);
truth = read_model_truth(model_name);
configure_adc_initial_conditions(model_name, truth);
set_param([model_name '/powergui'], 'SampleTime', ...
    num2str(options.PlantSampleTime, 17));

sil_process = start_sil_process(exe_path, project_dir);
process_cleanup = onCleanup(@() stop_sil_process(sil_process));
pause(0.5);
if sil_process.HasExited
    error('GMP:SilStartupFailed', 'SIL process exited before simulation start.');
end

sim_input = Simulink.SimulationInput(model_name);
sim_input = sim_input.setModelParameter( ...
    'StopTime', num2str(options.StopTime, 17), ...
    'SimulationMode', char(options.SimulationMode), ...
    'SolverType', 'Variable-step', ...
    'Solver', 'VariableStepAuto', ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', ...
    'ReturnWorkspaceOutputs', 'on');
sim_output = sim(sim_input);

logs = sim_output.logsout;
monitor = cell(1, numel(monitor_names));
for index = 1:numel(monitor_names)
    element = logs.getElement(monitor_names{index});
    monitor{index} = element.Values;
end

final_values = cellfun(@last_finite_value, monitor);
parameter_names = {'Rs'; 'Ld'; 'Lq'; 'FluxLinkage'; 'DeadTimeCompensation'};
model_values = [truth.Rs; truth.Ld; truth.Lq; truth.FluxLinkage; ...
    truth.ExpectedDeadTimeCompensation];
sil_values = final_values(2:6).';
units = {'ohm'; 'H'; 'H'; 'Wb'; 'V'};
relative_error_pct = 100.0 * (sil_values - model_values) ./ model_values;
relative_error_pct(~isfinite(relative_error_pct)) = NaN;

comparison = table(parameter_names, model_values, sil_values, units, ...
    relative_error_pct, 'VariableNames', ...
    {'Parameter', 'ModelValue', 'SILValue', 'Unit', 'RelativeErrorPct'});

result = struct();
result.model = model_path;
result.stop_time = options.StopTime;
result.final_state = final_values(1);
result.completed = (round(result.final_state) == 8);
result.comparison = comparison;
result.encoder = struct( ...
    'substate', final_values(7), ...
    'fault', final_values(8), ...
    'pole_pairs', final_values(10), ...
    'offset_pu', final_values(11));
result.mechanical = struct( ...
    'substate', final_values(9), ...
    'inertia_kg_m2', final_values(12), ...
    'viscous_friction_Nm_s', final_values(13), ...
    'load_torque_Nm', final_values(14));
result.monitor_names = monitor_names;
result.monitor = monitor;
result.model_truth = truth;
result.plant_sample_time = options.PlantSampleTime;
result.deadtime_resolved = (options.PlantSampleTime <= truth.DeadbandSeconds);
if isfile(trace_path)
    result.controller_state_trace = fileread(trace_path);
else
    result.controller_state_trace = '';
end

fprintf('\nPMSM offline-identification SIL comparison\n');
disp(comparison);
fprintf('Final OID state: %.0f (%s)\n', result.final_state, ...
    completed_text(result.completed));
if ~result.deadtime_resolved
    fprintf(['Fast plant mode: Ts=%.3g s is larger than the %.3g s deadband. ' ...
        'Use PlantSampleTime=1e-6 for final dead-time correlation.\n'], ...
        options.PlantSampleTime, truth.DeadbandSeconds);
end

if ~result.completed
    warning('GMP:IdentificationIncomplete', ...
        ['Identification did not reach COMPLETE (8). Increase StopTime or ' ...
         'inspect encoder substate/fault channels 7-8 and mechanical substate channel 9.']);
end

if options.Plot
    plot_sil_result(monitor, truth);
end

result_file = char(options.ResultFile);
if ~isempty(result_file)
    if ~is_absolute_path(result_file)
        result_file = fullfile(project_dir, result_file);
    end
    result_dir = fileparts(result_file);
    if ~isempty(result_dir) && ~isfolder(result_dir)
        mkdir(result_dir);
    end
    save(result_file, 'result');
    fprintf('Saved result: %s\n', result_file);
end
end

function build_sil_controller(solution_path)
vswhere = fullfile(getenv('ProgramFiles(x86)'), 'Microsoft Visual Studio', ...
    'Installer', 'vswhere.exe');
if ~isfile(vswhere)
    error('GMP:MissingVswhere', 'Visual Studio vswhere.exe was not found.');
end

[status, installation_path] = system(sprintf( ...
    '"%s" -latest -products * -requires Microsoft.Component.MSBuild -property installationPath', ...
    vswhere));
installation_path = strtrim(installation_path);
if status ~= 0 || isempty(installation_path)
    error('GMP:MissingMsbuild', 'A Visual Studio installation with MSBuild was not found.');
end

msbuild = fullfile(installation_path, 'MSBuild', 'Current', 'Bin', 'MSBuild.exe');
command = sprintf(['"%s" "%s" /m /t:Build /p:Configuration=Debug ' ...
    '/p:Platform=x64 /nologo /v:minimal'], msbuild, solution_path);
[status, output] = system(command, '-echo');
if status ~= 0
    error('GMP:SilBuildFailed', 'SIL controller build failed:\n%s', output);
end
end

function monitor_names = configure_monitor_logging(model_name)
monitor_names = arrayfun(@(index) sprintf('sil_monitor_%02d', index), ...
    1:16, 'UniformOutput', false);
selectors = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'BusSelector');
configured = false(1, 16);

for block_index = 1:numel(selectors)
    signal_names = strsplit(get_param(selectors{block_index}, 'OutputSignals'), ',');
    ports = get_param(selectors{block_index}, 'PortHandles');
    for port_index = 1:min(numel(signal_names), numel(ports.Outport))
        token = regexp(strtrim(signal_names{port_index}), ...
            '^Monitor CH(\d+)$', 'tokens', 'once');
        if isempty(token)
            continue;
        end
        channel = str2double(token{1});
        line = get_param(ports.Outport(port_index), 'Line');
        if channel < 1 || channel > 16 || line == -1
            continue;
        end
        set_param(ports.Outport(port_index), ...
            'DataLogging', 'on', ...
            'DataLoggingNameMode', 'Custom', ...
            'DataLoggingName', monitor_names{channel});
        configured(channel) = true;
    end
end

if ~all(configured)
    missing = find(~configured);
    error('GMP:MonitorContract', ...
        'Model is missing SIL monitor channels: %s', num2str(missing));
end
end

function truth = read_model_truth(model_name)
motor_block = [model_name '/Permanent Magnet' newline 'Synchronous Machine'];
dq_inductances = sscanf(strrep(strrep( ...
    get_param(motor_block, 'dqInductances'), '[', ''), ']', ''), '%f,%f');
if numel(dq_inductances) ~= 2
    error('GMP:MotorMask', 'Cannot parse motor dqInductances from the model mask.');
end

truth = struct();
truth.Rs = str2double(get_param(motor_block, 'Resistance'));
truth.Ld = dq_inductances(1);
truth.Lq = dq_inductances(2);
truth.FluxLinkage = str2double(get_param(motor_block, 'Flux'));
truth.Inertia = optional_mask_number(motor_block, 'Inertia');
truth.ViscousFriction = optional_mask_number(motor_block, 'Friction');
truth.PolePairs = optional_mask_number(motor_block, 'PolePairs');
truth.DeadbandSeconds = 1e-6 * str2double(get_param( ...
    [model_name '/Three Phase Driver Model (Universal)1'], 'Deadband_us'));
truth.DcBusVoltage = str2double(get_param([model_name '/Constant3'], 'Value'));
driver = [model_name '/Three Phase Driver Model (Universal)1'];
truth.SwitchingFrequency = str2double(get_param(driver, 'fswitch'));
truth.DiodeForwardVoltage = str2double(get_param(driver, 'Vf'));
% Six static current-vector directions produce a 4/3 Clarke projection;
% the OID two-phase voltage convention contributes the remaining factor 2.
truth.ExpectedDeadTimeCompensation = (8.0 / 3.0) * ...
    (truth.DcBusVoltage + 2.0 * truth.DiodeForwardVoltage) * ...
    truth.DeadbandSeconds * truth.SwitchingFrequency;
end

function value = optional_mask_number(block, parameter)
try
    value = str2double(get_param(block, parameter));
catch
    value = NaN;
end
end

function configure_adc_initial_conditions(model_name, truth)
% The detailed driver outputs its mask's ADC_INIT values before the first
% PWM sampling event.  Raw zero is not zero current for a biased ADC and can
% trip the controller's five-sample over-current debounce at t=0.  Seed the
% two sensed quantities with physically consistent raw codes instead.
driver = [model_name '/Three Phase Driver Model (Universal)1'];
adc_bits = str2double(get_param(driver, 'ADC_BIT'));
adc_reference = str2double(get_param(driver, 'ADC_REFERENCE'));
adc_bias = str2double(get_param(driver, 'ADC_BIAS'));
current_zero_code = round((adc_bias / adc_reference) * 2^adc_bits);

dc_bits = str2double(get_param(driver, 'DCBUS_ADC_BIT'));
dc_reference = str2double(get_param(driver, 'DCBUS_ADC_REFERENCE'));
dc_gain = str2double(get_param(driver, 'DCBUS_ADC_GAIN'));
dc_bias = str2double(get_param(driver, 'DCBUS_ADC_BIAS'));
dc_bus_code = round(((truth.DcBusVoltage * dc_gain + dc_bias) / ...
    dc_reference) * 2^dc_bits);

set_param(driver, ...
    'ADC_INIT', num2str(current_zero_code), ...
    'DCBUS_ADC_INIT', num2str(dc_bus_code));
end

function process = start_sil_process(exe_path, work_dir)
start_info = System.Diagnostics.ProcessStartInfo();
start_info.FileName = exe_path;
start_info.WorkingDirectory = work_dir;
start_info.UseShellExecute = false;
start_info.CreateNoWindow = true;
process = System.Diagnostics.Process();
process.StartInfo = start_info;
if ~process.Start()
    error('GMP:SilStartupFailed', 'Failed to start SIL process: %s', exe_path);
end
end

function stop_sil_process(process)
if isempty(process)
    return;
end
try
    if ~process.HasExited
        process.Kill();
        process.WaitForExit(5000);
    end
catch
    % Cleanup must not hide the simulation or identification error.
end
end

function value = last_finite_value(series)
data = series.Data(:);
index = find(isfinite(data), 1, 'last');
if isempty(index)
    value = NaN;
else
    value = double(data(index));
end
end

function text = completed_text(completed)
if completed
    text = 'COMPLETE';
else
    text = 'INCOMPLETE';
end
end

function plot_sil_result(monitor, truth)
figure('Name', 'PMSM Offline Identification SIL');
tiledlayout(3, 1);

nexttile;
stairs(monitor{1}.Time, monitor{1}.Data, 'LineWidth', 1.2);
grid on;
ylabel('OID state');
title('Identification progress');

nexttile;
hold on;
plot(monitor{2}.Time, monitor{2}.Data, 'DisplayName', 'Rs (ohm)');
plot(monitor{3}.Time, 1e3 * monitor{3}.Data, 'DisplayName', 'Ld (mH)');
plot(monitor{4}.Time, 1e3 * monitor{4}.Data, 'DisplayName', 'Lq (mH)');
plot(monitor{5}.Time, 1e3 * monitor{5}.Data, 'DisplayName', 'Flux (mWb)');
yline(truth.Rs, '--', 'Rs truth');
yline(1e3 * truth.Ld, '--', 'Ld truth');
yline(1e3 * truth.Lq, '--', 'Lq truth');
yline(1e3 * truth.FluxLinkage, '--', 'Flux truth');
grid on;
xlabel('Time (s)');
ylabel('Value (mixed displayed units)');
legend('Location', 'best');
title('SIL estimates and model truth');

nexttile;
yyaxis left;
plot(monitor{15}.Time, monitor{15}.Data, 'DisplayName', 'Speed (pu)');
ylabel('Mechanical speed (pu)');
yyaxis right;
stairs(monitor{16}.Time, monitor{16}.Data, 'DisplayName', 'PWM enabled');
ylabel('PWM enabled');
grid on;
xlabel('Time (s)');
title('Sensored acceleration and PWM-off coast-down');
end

function tf = is_absolute_path(path_value)
path_value = char(path_value);
tf = ~isempty(regexp(path_value, '^[A-Za-z]:[\\/]|^\\\\', 'once'));
end
