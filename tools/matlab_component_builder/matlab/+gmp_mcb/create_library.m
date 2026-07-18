function libraryPath = create_library()
% Create a masked Simulink library from the generated registry.
toolRoot = gmp_mcb.tool_root();
registry = gmp_mcb.load_registry();
installDir = fullfile(toolRoot, 'install', version('-release'));
if ~isfolder(installDir), mkdir(installDir); end
libraryName = 'gmp_mcb_components';
libraryPath = fullfile(installDir, [libraryName '.slx']);

if bdIsLoaded(libraryName), close_system(libraryName, 0); end
if isfile(libraryPath), delete(libraryPath); end
new_system(libraryName, 'Library');
cleanup = onCleanup(@() close_if_loaded(libraryName));
set_param(libraryName, 'Lock', 'off');

x = 60;
y = 60;
for index = 1:numel(registry.components)
    if iscell(registry.components)
        component = registry.components{index};
    else
        component = registry.components(index);
    end
    blockName = matlab.lang.makeValidName(char(component.display_name));
    block = [libraryName '/' blockName];
    add_block('built-in/S-Function', block, ...
        'FunctionName', char(component.sfunction_name), ...
        'Parameters', gmp_mcb.sfunction_parameter_expression(component), ...
        'Position', [x y x + 190 y + 100]);
    gmp_mcb.apply_component_mask(block, component);
    y = y + 150;
end

set_param(libraryName, 'EnableLBRepository', 'on');
save_system(libraryName, libraryPath);
close_system(libraryName, 0);
clear cleanup;

copyfile(fullfile(toolRoot, 'matlab', 'slblocks.m'), fullfile(installDir, 'slblocks.m'), 'f');
end

function close_if_loaded(name)
if bdIsLoaded(name), close_system(name, 0); end
end
