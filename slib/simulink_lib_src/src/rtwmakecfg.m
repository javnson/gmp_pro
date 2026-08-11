function makeInfo = rtwmakecfg()
%RTWMAKECFG Supply GMP SIL headers to generated simulation targets.
% The Simulink library distributes only the MATLAB-release-compatible MEX.
% Canonical C++ sources remain under tools/gmp_sil/sil_helper and are built
% by build_gmp_sil_mex.m, not duplicated in slib or listed as S-function
% source modules.

gmpRoot = string(getenv('GMP_PRO_LOCATION'));
if strlength(gmpRoot) == 0 || ~isfolder(gmpRoot)
    error('GMP:SIL:MissingRepositoryRoot', ...
        ['GMP_PRO_LOCATION must name the GMP repository before building ', ...
         'an Accelerator or Rapid Accelerator target.']);
end

switch computer('arch')
    case 'win64'
        triplet = 'x64-windows';
    case 'glnxa64'
        triplet = 'x64-linux';
    otherwise
        error('GMP:SIL:UnsupportedBuildArchitecture', ...
            'No GMP SIL vcpkg triplet is configured for %s.', computer('arch'));
end

% The GMP private-environment installer owns dependency restoration.  Its
% shared install root intentionally contains the triplet twice because
% VCPKG_INSTALLED_DIR is project-scoped by the environment manager.
dependencyInclude = fullfile(gmpRoot, 'bin', 'vcpkg_installed', ...
    triplet, triplet, 'include');
requiredHeaders = [ ...
    fullfile(dependencyInclude, 'asio.hpp'), ...
    fullfile(dependencyInclude, 'nlohmann', 'json.hpp')];
if ~all(isfile(requiredHeaders))
    error('GMP:SIL:MissingVcpkgDependencies', ...
        ['GMP SIL build dependencies are missing from %s. Run ', ...
         'tools/gmp_installer/repair_gmp_vcpkg.bat (or reinstall the GMP ', ...
         'private environment) before installing the Simulink library.'], ...
        dependencyInclude);
end

makeInfo.includePath = cellstr([gmpRoot, dependencyInclude]);
makeInfo.sourcePath = {};
makeInfo.sources = {};
makeInfo.linkLibsObjs = {};
end
