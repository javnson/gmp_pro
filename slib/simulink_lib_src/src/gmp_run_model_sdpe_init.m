function scripts = gmp_run_model_sdpe_init(model_reference)
%GMP_RUN_MODEL_SDPE_INIT Load common and project SDPE scripts for a model.
% Paths are derived at run time from the model location.  The expected tree is:
%   <suite>/sdpe_general/*_matlab_init.m
%   <suite>/project/<target>/sdpe_mgr/*_matlab_init.m

if nargin < 1 || isempty(model_reference)
    model_reference = bdroot;
end
model_file = resolve_model_file(model_reference);
model_dir = fileparts(model_file);
general_dir = fullfile(model_dir, '..', '..', 'sdpe_general');
project_dir = fullfile(model_dir, 'sdpe_mgr');

general_script = unique_init_script(general_dir, 'general');
project_script = unique_init_script(project_dir, 'project');
scripts = {general_script, project_script};

for index = 1:numel(scripts)
    script = scripts{index};
    addpath(fileparts(script));
    escaped = strrep(script, '''', '''''');
    evalin('base', sprintf('run(''%s'');', escaped));
end
end

function model_file = resolve_model_file(reference)
if isnumeric(reference)
    reference = get_param(reference, 'Name');
end
reference = char(string(reference));
if isfile(reference)
    model_file = char(java.io.File(reference).getCanonicalPath());
    return;
end
try
    model_file = get_param(reference, 'FileName');
catch
    model_file = '';
end
if isempty(model_file)
    model_file = which(reference);
end
if isempty(model_file)
    model_file = which([reference '.slx']);
end
if isempty(model_file)
    error('GMP:SDPE:ModelLocationUnknown', ...
        'Cannot determine the file location for model "%s".', reference);
end
model_file = char(java.io.File(model_file).getCanonicalPath());
end

function script = unique_init_script(folder, layer)
folder = char(java.io.File(folder).getCanonicalPath());
if ~isfolder(folder)
    error('GMP:SDPE:ManagerMissing', ...
        'The %s SDPE folder does not exist relative to the model: %s', layer, folder);
end
matches = dir(fullfile(folder, '*_matlab_init.m'));
matches = matches(~[matches.isdir]);
if numel(matches) ~= 1
    error('GMP:SDPE:InitScriptCount', ...
        'Expected exactly one generated MATLAB init script in %s, found %d.', ...
        folder, numel(matches));
end
script = fullfile(matches(1).folder, matches(1).name);
end
