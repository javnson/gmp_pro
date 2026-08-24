% This function may remove GMP Simulink Library 

function uninstall_gmp_simulink_lib()

matlab_path = fileparts(mfilename('fullpath'));
component_uninstaller = fullfile(matlab_path, 'ctl_simulink_components', ...
    'matlab', 'uninstall_ctl_simulink_components.m');
if isfile(component_uninstaller)
    run(component_uninstaller);
else
    warning('GMP:Simulink:MissingCtlComponentsUninstaller', ...
        'CTL Simulink Components uninstaller is missing: %s', ...
        component_uninstaller);
end

%% remove MATLAB path
disp('Remove MATLAB path');

matlab_version = matlabRelease; %matlab_version.Release => R2022b
simulink_lib_path = fullfile(matlab_path, 'install_path', matlab_version.Release);

if contains(path, simulink_lib_path), rmpath(simulink_lib_path); end

m_file_path = fullfile(simulink_lib_path, 'src');
if contains(path, m_file_path), rmpath(m_file_path); end

% Persist the removal so deleted GMP paths are not restored in the next
% MATLAB session.
savepath;

%% remove files
disp('Remove Simlink Related files.');
if isfolder(simulink_lib_path), rmdir(simulink_lib_path, 's'); end

disp('GMP Simulink Library is uninstalled successfully.');
end
