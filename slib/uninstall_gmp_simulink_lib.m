% This function may remove GMP Simulink Library 

function uninstall_gmp_simulink_lib()

clear all; %#ok

%% remove MATLAB path
disp('Remove MATLAB path');

matlab_version = matlabRelease; %matlab_version.Release => R2022b
matlab_path = fileparts(mfilename('fullpath'));
simulink_lib_path = append(fullfile(matlab_path), '\install_path\', matlab_version.Release);

rmpath(simulink_lib_path);

m_file_path = append(simulink_lib_path, '/src');
rmpath(m_file_path);

% Persist the removal so deleted GMP paths are not restored in the next
% MATLAB session.
savepath;

%% remove files
disp('Remove Simlink Related files.');
rmdir(simulink_lib_path, 's');

disp('GMP Simulink Library is uninstalled successfully.');
end
