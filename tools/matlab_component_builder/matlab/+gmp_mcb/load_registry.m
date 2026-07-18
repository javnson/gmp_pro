function registry = load_registry()
% Load generated component metadata.
path = fullfile(gmp_mcb.tool_root(), 'build', 'registry.json');
if ~isfile(path)
    error('GMP:MCB:MissingRegistry', ...
        'Generated registry is missing. Run matlab_component_builder.py generate first.');
end
registry = jsondecode(fileread(path));
end

