function simulation_output = run_pil_stage(expected_build_level, stop_time)
%RUN_PIL_STAGE Run the FSBB Simulink plant through the GMP Data Link PIL bridge.
arguments
    expected_build_level (1,1) double {mustBeMember(expected_build_level, 1:3)}
    stop_time (1,1) double {mustBePositive, mustBeFinite}
end

source_directory = fileparts(fileparts(mfilename('fullpath')));
project_directory = fileparts(fileparts(source_directory));
run(fullfile(source_directory, 'sdpe_mgr', 'ctrl_settings_matlab_init.m'));
assert(exist('ENABLE_GMP_DL_PIL_SIM', 'var') && ENABLE_GMP_DL_PIL_SIM, 'FSBB:PILDisabled', ...
    'Enable ENABLE_GMP_DL_PIL_SIM and regenerate the target SDPE settings.');
assert(BUILD_LEVEL == expected_build_level, 'FSBB:PILBuildLevelMismatch', ...
    'The target SDPE selects level %d, but level %d was requested.', BUILD_LEVEL, expected_build_level);

release_directory = fullfile(getenv('GMP_PRO_LOCATION'), 'slib', ...
    'install_path', ['R' version('-release')]);
assert(isfolder(release_directory), 'FSBB:PILLibraryMissing', ...
    'Install the GMP Simulink libraries for this MATLAB release.');
addpath(release_directory, fullfile(release_directory, 'src'));
model = 'MCS_STD_FSBB_MODEL';
assert(~bdIsLoaded(model), 'FSBB:PILModelAlreadyOpen', ...
    'Close %s before running PIL so the runner cannot change an open SIL session.', model);
model_directory = fullfile(project_directory, 'simulate');
assert(isfile(fullfile(model_directory, 'sdpe_mgr', 'ctrl_settings_matlab_init.m')), ...
    'FSBB:PILPlantSettingsMissing', 'Generate the simulate project SDPE settings before running PIL.');
network_path = fullfile(model_directory, 'network.json');
network_backup = '';
if isfile(network_path)
    network_backup = [tempname(model_directory) '.json'];
    copyfile(network_path, network_backup);
end
network_cleanup = onCleanup(@() restore_network(network_path, network_backup)); %#ok<NASGU>
load_system(fullfile(model_directory, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>

panel = [model '/GMP Stanrdard Digital Power Controller Panel (SIL) Pro Edition'];
set_param(panel, 'LinkStatus', 'inactive');
core = [panel '/GMP SIL Core With trigger/GMP SIL Core Module'];
expected = {'MaskPackedDataTypes', '{''double'',''uint32'',''double'',''int32''}'; ...
            'MaskPackedDataSizes', '{[1],[24],[16],[8]}'; ...
            'MaskUnpackedDataTypes', '{''double'',''uint32'',''uint32'',''double''}'; ...
            'MaskUnpackedDataSizes', '{[1],[8],[8],[16]}'};
for index = 1:size(expected, 1)
    actual = regexprep(get_param(core, expected{index, 1}), '\s+', '');
    assert(strcmp(actual, expected{index, 2}), 'FSBB:PILPackingMismatch', ...
        '%s is incompatible with the GMP PIL bridge.', expected{index, 1});
end
set_param(core, ...
    'MaskTransport', 'UDP', ...
    'MaskHostAddr', sprintf('''%s''', GMP_PIL_UDP_HOST), ...
    'MaskMsgTxPort', num2str(GMP_PIL_BRIDGE_UDP_LISTEN_PORT), ...
    'MaskMsgRxPort', num2str(GMP_PIL_MATLAB_UDP_LISTEN_PORT));
% The saved model loads its 3000-count SIL settings in InitFcn. Use the
% target's TBPRD and deadband explicitly for this in-memory PIL run.
set_param([model '/GMP STD FSBB Module'], ...
    'PWM_Compare_Max', num2str(CTRL_PWM_CMP_MAX), ...
    'Deadband_Compare', num2str(CTRL_PWM_DEADBAND_CMP));

simulation_output = sim(model, 'StopTime', num2str(stop_time, 17), ...
    'ReturnWorkspaceOutputs', 'on');
clear cleanup;
end

function restore_network(path, backup)
if ~isempty(backup)
    movefile(backup, path, 'f');
elseif isfile(path)
    delete(path);
end
end
