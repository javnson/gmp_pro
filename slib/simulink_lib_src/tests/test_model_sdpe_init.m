function tests = test_model_sdpe_init
tests = functiontests(localfunctions);
end

function testRelativeTwoLayerInitialization(testCase)
root = tempname; mkdir(root);
cleanup = onCleanup(@() cleanup_tree(root));
model_dir = fullfile(root, 'suite', 'project', 'simulate'); mkdir(model_dir);
general_dir = fullfile(root, 'suite', 'sdpe_general'); mkdir(general_dir);
project_dir = fullfile(model_dir, 'sdpe_mgr'); mkdir(project_dir);
model_file = fullfile(model_dir, 'test_model.slx'); fclose(fopen(model_file, 'w'));
write_script(fullfile(general_dir, 'common_matlab_init.m'), ...
    'GMP_TEST_SDPE_ORDER = "general";');
write_script(fullfile(project_dir, 'project_matlab_init.m'), ...
    'GMP_TEST_SDPE_ORDER = GMP_TEST_SDPE_ORDER + ",project";');
evalin('base', 'clear GMP_TEST_SDPE_ORDER');
scripts = gmp_run_model_sdpe_init(model_file);
verifyEqual(testCase, evalin('base', 'GMP_TEST_SDPE_ORDER'), "general,project");
verifyEqual(testCase, numel(scripts), 2);
evalin('base', 'clear GMP_TEST_SDPE_ORDER');
clear cleanup
end

function testRejectsAmbiguousGeneratedScripts(testCase)
root = tempname; mkdir(root);
cleanup = onCleanup(@() cleanup_tree(root));
model_dir = fullfile(root, 'suite', 'project', 'simulate'); mkdir(model_dir);
general_dir = fullfile(root, 'suite', 'sdpe_general'); mkdir(general_dir);
project_dir = fullfile(model_dir, 'sdpe_mgr'); mkdir(project_dir);
model_file = fullfile(model_dir, 'test_model.slx'); fclose(fopen(model_file, 'w'));
write_script(fullfile(general_dir, 'a_matlab_init.m'), 'A=1;');
write_script(fullfile(general_dir, 'b_matlab_init.m'), 'B=1;');
write_script(fullfile(project_dir, 'p_matlab_init.m'), 'P=1;');
verifyError(testCase, @() gmp_run_model_sdpe_init(model_file), ...
    'GMP:SDPE:InitScriptCount');
clear cleanup
end

function write_script(path, content)
fid = fopen(path, 'w'); cleaner = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', content); clear cleaner
end

function cleanup_tree(root)
entries = strsplit(path, pathsep);
for index = 1:numel(entries)
    if startsWith(entries{index}, root, 'IgnoreCase', ispc)
        rmpath(entries{index});
    end
end
if isfolder(root), rmdir(root, 's'); end
end
