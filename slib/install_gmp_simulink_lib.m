% This m file will instann GMP Simulink Utilities

function install_gmp_simulink_lib()

matlab_version = matlabRelease; %matlab_version.Release => R2022b
assert_specialized_power_systems_available(matlab_version.Release);

matlab_path = fileparts(mfilename('fullpath'));
simulink_lib_path = fullfile(matlab_path, 'install_path', matlab_version.Release);

% upgrade_gmp_simulink_lib uses repository-relative source paths.  Make the
% installer independent of the caller's current working directory.
installer_path = pwd;
restore_path = onCleanup(@() cd(installer_path));
cd(matlab_path);

dependency_include = ensure_gmp_sil_vcpkg_dependencies(matlab_path);
build_and_stage_gmp_sil_core(matlab_path, dependency_include);
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


function include_root = ensure_gmp_sil_vcpkg_dependencies(slib_path)
% Validate dependencies restored by the GMP private-environment installer.
% MATLAB intentionally never invokes vcpkg or performs network access.

repository_root = fileparts(slib_path);
switch computer('arch')
    case 'win64'
        triplet = 'x64-windows';
    case 'glnxa64'
        triplet = 'x64-linux';
    otherwise
        error('GMP:SIL:UnsupportedInstallArchitecture', ...
            'No GMP SIL vcpkg triplet is configured for %s.', computer('arch'));
end

include_root = fullfile(repository_root, 'bin', 'vcpkg_installed', ...
    triplet, triplet, 'include');
if isfile(fullfile(include_root, 'asio.hpp')) && ...
        isfile(fullfile(include_root, 'nlohmann', 'json.hpp'))
    return;
end

error('GMP:SIL:MissingVcpkgDependencies', ...
    ['GMP SIL headers are missing from %s. MATLAB did not modify the ', ...
     'toolchain. Run tools/gmp_installer/repair_gmp_vcpkg.bat from ', ...
     'gmp_env.bat, or reinstall the GMP private environment, then retry.'], ...
    include_root);
end


function build_and_stage_gmp_sil_core(slib_path, dependency_include)
% Compile from the canonical tools source, then stage only the MEX binary
% in slib.  This prevents the Simulink package from becoming a second C++
% source tree.

repository_root = fileparts(slib_path);
helper_path = fullfile(repository_root, 'tools', 'gmp_sil', 'sil_helper');
builder = fullfile(helper_path, 'build_gmp_sil_mex.m');
if ~isfile(builder)
    error('GMP:SIL:MissingMexBuilder', ...
        'The GMP SIL MATLAB MEX builder is missing: %s', builder);
end

build_directory = tempname;
mkdir(build_directory);
remove_build_directory = onCleanup(@() rmdir(build_directory, 's'));
original_path = path;
restore_path = onCleanup(@() path(original_path));
addpath(helper_path, '-begin');

artifact = build_gmp_sil_mex( ...
    'RepositoryRoot', repository_root, ...
    'DependencyInclude', dependency_include, ...
    'OutputDirectory', build_directory);
destination_directory = fullfile(slib_path, 'simulink_lib_src', 'src');
destination = fullfile(destination_directory, ['GMP_SIL_Core.' mexext]);
clear('GMP_SIL_Core');
[copied, message] = copyfile(artifact, destination, 'f');
if ~copied
    error('GMP:SIL:MexStageFailed', ...
        'Could not copy the built GMP SIL MEX to %s:\n%s', ...
        destination, message);
end
fprintf('GMP Simulink Library: staged %s.\n', destination);
clear restore_path remove_build_directory;
end


function assert_specialized_power_systems_available(matlab_release)
% GMP Simulink models require the Specialized Power Systems block library.
% Test the actual library instead of relying only on product metadata: this
% also catches an installation that is incomplete or unavailable to MATLAB.

required_library = 'powerlib';
required_product = 'Simscape Electrical / Specialized Power Systems';

if isempty(which(required_library))
    release_number = str2double(extract(matlab_release, digitsPattern));
    if release_number >= 2026
        error('GMP:SpecializedPowerSystemsRemoved', ...
            ['GMP Simulink Library installation stopped.\n' ...
             'Specialized Power Systems was removed from MATLAB R2026a and later.\n' ...
             'Use MATLAB R2025b or earlier with %s installed.'], ...
            required_product);
    end

    error('GMP:MissingSpecializedPowerSystems', ...
        ['GMP Simulink Library installation stopped.\n' ...
         'The required library "%s" was not found.\n' ...
         'Install %s with the MathWorks installer, restart MATLAB, and run this installer again.'], ...
        required_library, required_product);
end

was_loaded = bdIsLoaded(required_library);
try
    load_system(required_library);
catch load_error
    if ~was_loaded && bdIsLoaded(required_library)
        close_system(required_library, 0);
    end
    error('GMP:SpecializedPowerSystemsUnavailable', ...
        ['GMP Simulink Library installation stopped.\n' ...
         '%s is installed but MATLAB could not load "%s".\n' ...
         'Check the product license and installation, then try again.\n' ...
         'MATLAB details: %s'], ...
        required_product, required_library, load_error.message);
end

if ~was_loaded
    close_system(required_library, 0);
end
end
