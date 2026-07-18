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
    component = registry.components(index);
    blockName = matlab.lang.makeValidName(char(component.display_name));
    block = [libraryName '/' blockName];
    add_block('built-in/S-Function', block, ...
        'FunctionName', char(component.sfunction_name), ...
        'Parameters', ['gmp_mcb.init_mode_code(init_method),' ...
            'kp,ki_or_ti,kd_or_td,fs,out_max,out_min,integral_max,integral_min'], ...
        'Position', [x y x + 190 y + 100]);
    gmp_mcb.apply_pid_mask(block, component);
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

