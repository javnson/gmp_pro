function uninstall()
% Remove CTL component paths and its independent Release installation.
toolRoot = gmp_mcb.tool_root();
matlabRoot = fullfile(toolRoot, 'matlab');
paths = gmp_mcb.install_paths();

if bdIsLoaded('ctl_simulink_components')
    close_system('ctl_simulink_components', 0);
end
if contains(path, paths.installDir), rmpath(paths.installDir); end
if contains(path, paths.mexDir), rmpath(paths.mexDir); end
if contains(path, matlabRoot), rmpath(matlabRoot); end
savepath;

if isfolder(paths.installDir)
    mexFiles = dir(fullfile(paths.mexDir, ['gmp_mcb_*.' mexext]));
    for index = 1:numel(mexFiles)
        mexFunction = erase(mexFiles(index).name, ['.' mexext]);
        clear(mexFunction);
    end
    rmdir(paths.installDir, 's');
end

rehash toolboxcache;
sl_refresh_customizations;
fprintf('CTL Simulink Components removed for %s.\n', paths.release);
end
