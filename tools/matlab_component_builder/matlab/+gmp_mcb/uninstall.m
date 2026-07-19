function uninstall()
% Remove registered paths. Generated files remain available for inspection.
toolRoot = gmp_mcb.tool_root();
matlabRoot = fullfile(toolRoot, 'matlab');
releaseDir = fullfile(toolRoot, 'install', version('-release'));
mexDir = fullfile(releaseDir, 'mex');
if contains(path, releaseDir), rmpath(releaseDir); end
if contains(path, mexDir), rmpath(mexDir); end
if contains(path, matlabRoot), rmpath(matlabRoot); end
savepath;
rehash toolboxcache;
sl_refresh_customizations;
fprintf('GMP MATLAB Components paths removed for %s.\n', version('-release'));
end
