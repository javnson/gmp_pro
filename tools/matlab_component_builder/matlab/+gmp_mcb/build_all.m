function paths = build_all()
% Compile every generated component S-Function for the active MATLAB Release.
toolRoot = gmp_mcb.tool_root();
gmpRoot = gmp_mcb.gmp_root();
registry = gmp_mcb.load_registry();
paths.installDir = fullfile(toolRoot, 'install', version('-release'));
paths.mexDir = fullfile(paths.installDir, 'mex');
if ~isfolder(paths.mexDir), mkdir(paths.mexDir); end

includeDir = fullfile(toolRoot, 'matlab', 'include');
for index = 1:numel(registry.components)
    component = registry.components(index);
    arguments = { ...
        '-R2018a', ...
        ['-I' includeDir], ...
        ['-I' gmpRoot], ...
        '-outdir', paths.mexDir, ...
        '-output', component.sfunction_name, ...
        char(component.generated_source)};
    sources = cellstr(string(component.implementation.sources));
    for sourceIndex = 1:numel(sources)
        arguments{end + 1} = fullfile(gmpRoot, sources{sourceIndex}); %#ok<AGROW>
    end
    fprintf('Compiling %s...\n', component.sfunction_name);
    mex(arguments{:});
    binary = fullfile(paths.mexDir, [char(component.sfunction_name) '.' mexext]);
    if ~isfile(binary)
        error('GMP:MCB:MissingMex', 'MEX build did not produce %s.', binary);
    end
end
end

