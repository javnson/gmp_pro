% This function may upgrade GMP Simulink Library
function upgrade_gmp_simulink_lib()

clear all; %#ok
bdclose;

%% validate MATLAB version
if (hex2dec(version('-release')) < hex2dec('2022b'))
	disp('Error: Please use Matlab 2022b or later version!');
	return ;
end

%% validate necessary files
if(~isfile('simulink_lib_src/gmp_simulink_utilities_src.slx'))
	disp('Error: gmp_simulink_utilities.slx Simulink model is missing!');
	return;
end

if(~isfile('simulink_lib_src/gmp_sil_core_pack_src.slx'))
	disp('Error: gmp_sil_core_pack_src.slx Simulink model is missing!');
	return;
end

if(~isfile('simulink_lib_src/gmp_peripheral_utilities_src.slx'))
	disp('Error: gmp_peripheral_utilities.slx Simulink model is missing!');
	return;
end

if(~isfile('simulink_lib_src/gmp_fp_utilities_src.slx'))
	disp('Error: gmp_fp_utilities.slx Simulink model is missing!');
	return;
end

if(~isfile('simulink_lib_src/gmp_std_model_pck_src.slx'))
	disp('Error: gmp_std_model_pck_src.slx Simulink model is missing!');
	return;
end

if(~isfile('simulink_lib_src/slblocks.m'))
	disp('Error: slblocks.m Simulink script is missing!');
	return;
end

%% Get target simulink library path

matlab_version = matlabRelease; %matlab_version.Release => R2022b
matlab_path = fileparts(mfilename('fullpath'));
simulink_lib_path = append(fullfile(matlab_path), '\install_path/', matlab_version.Release);

%% generate target version simulink model

disp('GMP Simulink Library: GMP Simulink Library is generating.');

if (~exist(append("install_path/",matlab_version.Release)))
   mkdir(append("install_path/",matlab_version.Release));
end

% Debug Switch
warning('off','all')


close_system('gmp_simulink_utilities.slx', 0);
close_system('gmp_simulink_utilities_src.slx', 0);
close_system('gmp_fp_utilities.slx', 0);
close_system('gmp_fp_utilities_src.slx', 0);
close_system('gmp_peripheral_utilities.slx', 0);
close_system('gmp_peripheral_utilities_src.slx', 0);
close_system('gmp_sil_core_pack.slx', 0);
close_system('gmp_sil_core_pack_src.slx', 0);
close_system('gmp_std_model_pck.slx', 0);
close_system('gmp_std_model_pck_src.slx', 0);
close_system('gmp_component_model.slx', 0);
close_system('gmp_component_model_src.slx', 0);


generate_single_slx_lib('gmp_simulink_utilities');
generate_single_slx_lib('gmp_fp_utilities');
generate_single_slx_lib('gmp_peripheral_utilities');
generate_single_slx_lib('gmp_sil_core_pack');
generate_single_slx_lib('gmp_std_model_pck');
generate_single_slx_lib('gmp_component_model');
 
% load_system('simulink_lib_src/gmp_simulink_utilities_src.slx');
% target_file = append('install_path/',matlab_version.Release,'/gmp_simulink_utilities.slx');
% %set_param('gmp_simulink_utilities_src','Lock','off')
% save_system('simulink_lib_src/gmp_simulink_utilities_src.slx',target_file);
% close_system('gmp_simulink_utilities.slx');
% % close_system('gmp_simulink_utilities_src.slx');
% 
% load_system('simulink_lib_src/fp_utilities_src.slx');
% target_file = append('install_path/',matlab_version.Release,'/fp_utilities.slx');
% %set_param('fp_utilities_src','Lock','off')
% save_system('simulink_lib_src/fp_utilities_src.slx',target_file);
% close_system('fp_utilities.slx');
% % close_system('fp_utilities_src.slx');
% 
% load_system('simulink_lib_src/peripheral_utilities_src.slx');
% target_file = append('install_path/',matlab_version.Release,'/peripheral_utilities.slx');
% %set_param('peripheral_utilities_src','Lock','off')
% save_system('simulink_lib_src/peripheral_utilities_src.slx',target_file);
% close_system('peripheral_utilities.slx');
% % close_system('peripheral_utilities_src.slx');

warning('on','all')

%% Compiling MEX Library

disp('Compiling MEX_UDP_Helper.');

% compile_cmd = sprintf('mex ''-I"E:\lib\gmp_pro\core\util\udp_helper"''')

%% Copy 3D Models for SIL 

% copy all m files
if (~exist(append(simulink_lib_path,'/avatars')))
    mkdir(append(simulink_lib_path,'/avatars'));
end

copyfile('simulink_lib_src/avatars', append(simulink_lib_path,'/avatars'), 'f');


%% Copy other files

disp('GMP Simulink Library: Other necessary files are copying.');

target_file = append('install_path/',matlab_version.Release,'/slblocks.m');
copyfile('simulink_lib_src/slblocks.m', target_file, 'f');

% copy all m files
if (~exist(append(simulink_lib_path,'/src')))
    mkdir(append(simulink_lib_path,'/src'));
end


clear GMP_SIL_Core
clear MEX_UDP_Helper

copyfile('simulink_lib_src/src', append(simulink_lib_path,'/src'), 'f');

% copy all icon files
if (~exist(append(simulink_lib_path,'/icon')))
    mkdir(append(simulink_lib_path,'/icon'));
end

copyfile('simulink_lib_src/icon', append(simulink_lib_path,'/icon'), 'f');

end % function end


%% static utility function
function generate_single_slx_lib(libname)
% Get matlab release version
matlab_version = matlabRelease;

% ensure all the libfile is closed
close_system(append(libname,'.slx'), 0);
close_system(append(libname,'_src.slx'), 0);

% generate lib file
load_system(append('simulink_lib_src/',libname, '_src.slx'));
target_file = append('install_path/',matlab_version.Release,'/',libname,'.slx');
set_param(append(libname,'_src'),'Lock','off')
save_system(append('simulink_lib_src/',libname, '_src.slx'), target_file);

matlab_version_str = extract(matlab_version.Release, digitsPattern);
if(str2double(matlab_version_str)>2023)
    close_system(libname, 1);
    close_system(append(libname,'_src'), 0);

else
    close_system(append(libname,'.slx'));
    close_system(append(libname,'_src'), 0);
end

end

