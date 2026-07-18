function install()
% Build, generate, and register the independent component library.
toolRoot = gmp_mcb.tool_root();
matlabRoot = fullfile(toolRoot, 'matlab');
addpath(matlabRoot);

registryPath = fullfile(toolRoot, 'build', 'registry.json');
if ~isfile(registryPath)
    pythonEntry = fullfile(toolRoot, 'matlab_component_builder.py');
    command = sprintf('python "%s" generate', pythonEntry);
    [status, output] = system(command);
    if status ~= 0
        error('GMP:MCB:GenerationFailed', 'Python generation failed:\n%s', output);
    end
end

paths = gmp_mcb.build_all();
libraryPath = gmp_mcb.create_library();
addpath(paths.installDir);
addpath(paths.mexDir);
savepath;
rehash toolboxcache;
sl_refresh_customizations;
fprintf('GMP MATLAB Components installed for %s.\nLibrary: %s\n', version('-release'), libraryPath);
end

