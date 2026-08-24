function paths = install_paths()
% Return the shared slib installation paths for the active MATLAB Release.
toolRoot = gmp_mcb.tool_root();
slibRoot = fileparts(toolRoot);
matlabVersion = matlabRelease;
paths.release = char(matlabVersion.Release);
paths.installRoot = fullfile(slibRoot, 'install_path');
paths.installDir = fullfile(paths.installRoot, ...
    'ctl_simulink_components', paths.release);
paths.mexDir = fullfile(paths.installDir, 'mex');
end
