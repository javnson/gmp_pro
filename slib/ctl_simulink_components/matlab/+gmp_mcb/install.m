function install()
% Build, generate, and register the CTL component library.
toolRoot = gmp_mcb.tool_root();
matlabRoot = fullfile(toolRoot, 'matlab');
addpath(matlabRoot);

generationOutput = gmp_mcb.generate_registry();
fprintf('%s', generationOutput);

paths = gmp_mcb.build_all();
libraryPath = gmp_mcb.create_library();
addpath(paths.installDir);
addpath(paths.mexDir);
savepath;
rehash toolboxcache;
sl_refresh_customizations;
fprintf('CTL Simulink Components installed for %s.\nLibrary: %s\n', paths.release, libraryPath);
end
