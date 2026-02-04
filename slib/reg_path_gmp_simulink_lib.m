% This m file will instann GMP Simulink Utilities

function reg_gmp_simulink_lib()

matlab_version = matlabRelease; %matlab_version.Release => R2022b
matlab_path = fileparts(mfilename('fullpath'));
%matlab_path = '%GMP_PRO_LOCATION%/slib';
simulink_lib_path = append(fullfile(matlab_path), '/install_path/', matlab_version.Release);

%% register model path

addpath(simulink_lib_path);

m_file_path = append(simulink_lib_path, '/src');
addpath(m_file_path);

savepath;

%% enable Simulink Model Library

disp('GMP Simulink Library: Register to Simulink Library');

%% Complete
disp('GMP Simulink Library has registered Successfully.');


end % function end
