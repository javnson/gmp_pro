function export_gmp_simulink_lib_src()
%EXPORT_GMP_SIMULINK_LIB_SRC Publish the R2022b-compatible SIL source library.
% The editable/debugged library is install_path/R2024b/gmp_sil_core_pack.slx.
% This function applies the current SIL mask schema to a temporary R2024b
% copy, exports it with Simulink.exportToVersion, and atomically publishes
% the resulting R2022b model into simulink_lib_src.

slibPath = fileparts(mfilename('fullpath'));
r2024bLibrary = fullfile(slibPath, 'install_path', 'R2024b', ...
    'gmp_sil_core_pack.slx');
sourceLibrary = fullfile(slibPath, 'simulink_lib_src', ...
    'gmp_sil_core_pack_src.slx');
helperPath = fullfile(slibPath, 'simulink_lib_src', 'src');

if ~isfile(r2024bLibrary)
    error('GMP:SIL:MissingR2024bLibrary', ...
        'The editable R2024b SIL library is missing: %s', r2024bLibrary);
end
releaseParts = regexp(version('-release'), '^R?(\d{4})([ab])$', ...
    'tokens', 'once');
if isempty(releaseParts)
    error('GMP:SIL:UnknownMatlabRelease', ...
        'Cannot interpret MATLAB Release name: %s', version('-release'));
end
releaseOrdinal = 2 * str2double(releaseParts{1}) + ...
    double(strcmpi(releaseParts{2}, 'b'));
if releaseOrdinal < 2 * 2024 + 1
    error('GMP:SIL:ExportRequiresR2024b', ...
        'Use MATLAB R2024b or later to export the R2024b SIL library.');
end

temporaryLibrary = [tempname(slibPath), '.slx'];
cleanupTemporary = onCleanup(@() deleteIfPresent(temporaryLibrary));
[copied, message] = copyfile(r2024bLibrary, temporaryLibrary, 'f');
if ~copied
    error('GMP:SIL:ExportCopyFailed', ...
        'Could not prepare the R2024b SIL library for export: %s', message);
end

originalPath = path;
restorePath = onCleanup(@() path(originalPath));
addpath(helperPath, '-begin');
upgrade_gmp_sil_core_transport_mask(temporaryLibrary);

[published, message] = movefile(temporaryLibrary, sourceLibrary, 'f');
if ~published
    error('GMP:SIL:ExportPublishFailed', ...
        'Could not publish the R2022b SIL source library: %s', message);
end
fprintf('GMP Simulink Library: exported R2024b SIL library to R2022b source:\n  %s\n', ...
    sourceLibrary);
clear restorePath cleanupTemporary;
end

function deleteIfPresent(path)
if isfile(path)
    delete(path);
end
end
