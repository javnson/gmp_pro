% This m file will instann GMP Simulink Utilities

function install_gmp_simulink_lib()

matlab_version = matlabRelease; %matlab_version.Release => R2022b
matlab_path = fileparts(mfilename('fullpath'));
simulink_lib_path = fullfile(matlab_path, 'install_path', matlab_version.Release);

% upgrade_gmp_simulink_lib uses repository-relative source paths.  Make the
% installer independent of the caller's current working directory.
installer_path = pwd;
restore_path = onCleanup(@() cd(installer_path));
cd(matlab_path);

upgrade_gmp_simulink_lib();

%% register model path

addpath(simulink_lib_path);

m_file_path = fullfile(simulink_lib_path, 'src');
addpath(m_file_path);

savepath;

%% enable Simulink Model Library

disp('GMP Simulink Library: Register to Simulink Library');

cd(simulink_lib_path)

load_system('gmp_peripheral_utilities');
set_param('gmp_peripheral_utilities','Lock','off')

load_system('gmp_fp_utilities');
set_param('gmp_fp_utilities','Lock','off')

load_system('gmp_sil_core_pack');
set_param('gmp_sil_core_pack','Lock','off')

load_system('gmp_simulink_utilities');
set_param('gmp_simulink_utilities','Lock','off')

load_system('gmp_std_model_pck');
set_param('gmp_std_model_pck','Lock','off')

set_param(gcs,'EnableLBRepository','on');
sl_refresh_customizations

matlab_version_str = extract(matlab_version.Release, digitsPattern);
if(str2double(matlab_version_str) >= 2023)
    close_system('gmp_peripheral_utilities', 1);
    close_system('gmp_fp_utilities', 1);
    close_system('gmp_simulink_utilities', 1);
    close_system('gmp_sil_core_pack', 1);
else
    close_system('gmp_peripheral_utilities');
    close_system('gmp_fp_utilities');
    close_system('gmp_simulink_utilities');
    close_system('gmp_sil_core_pack');
end


%% Complete
disp('GMP Simulink Library has installed Successfully.');


end % function end
