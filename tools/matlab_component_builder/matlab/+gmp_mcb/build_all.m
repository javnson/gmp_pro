function paths = build_all(outputDir)
% Compile every generated component S-Function for the active MATLAB Release.
toolRoot = gmp_mcb.tool_root();
gmpRoot = gmp_mcb.gmp_root();
registry = gmp_mcb.load_registry();
if nargin < 1 || strlength(string(outputDir)) == 0
    paths.installDir = fullfile(toolRoot, 'install', version('-release'));
    paths.mexDir = fullfile(paths.installDir, 'mex');
else
    paths.installDir = '';
    paths.mexDir = char(outputDir);
end
if ~isfolder(paths.mexDir), mkdir(paths.mexDir); end

includeDir = fullfile(toolRoot, 'matlab', 'include');
for index = 1:numel(registry.components)
    if iscell(registry.components)
        component = registry.components{index};
    else
        component = registry.components(index);
    end
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
