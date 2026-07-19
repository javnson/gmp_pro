function output = generate_registry()
% Generate the component registry using the canonical GMP Python environment.
toolRoot = gmp_mcb.tool_root();
launcher = fullfile(toolRoot, 'run_python.bat');
if ~isfile(launcher)
    error('GMP:MCB:MissingLauncher', 'Python launcher does not exist: %s', launcher);
end

command = sprintf('"%s" generate', launcher);
[status, output] = system(command);
if status ~= 0
    error('GMP:MCB:GenerationFailed', ...
        ['Python generation failed through the GMP environment selector.\n%s\n' ...
         'Repair the selected environment with install_gmp_virtual_env.bat ' ...
         'or install_gmp.bat.'], output);
end

registryPath = fullfile(toolRoot, 'build', 'registry.json');
if ~isfile(registryPath)
    error('GMP:MCB:MissingRegistry', ...
        'Generation reported success but did not create %s.', registryPath);
end
end
