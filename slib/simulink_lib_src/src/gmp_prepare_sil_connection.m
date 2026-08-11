function ConnectionId = gmp_prepare_sil_connection(blockPath, transportName, ...
    targetAddress, transmitPort, receivePort, requestBytes, responseBytes)
%GMP_PREPARE_SIL_CONNECTION Generate the controller-side SIL session JSON.
% The connection ID deterministically binds the saved model, block path,
% transport, ports and packed ABI. A copied model therefore gets a different
% identity while repeated diagram updates keep one stable controller contract.

arguments
    blockPath {mustBeTextScalar}
    transportName {mustBeTextScalar}
    targetAddress {mustBeTextScalar}
    transmitPort (1,1) double
    receivePort (1,1) double
    requestBytes (1,1) double
    responseBytes (1,1) double
end

rootModel = bdroot(blockPath);
modelFile = string(get_param(rootModel, 'FileName'));
if strlength(modelFile) == 0
    identityPath = string(rootModel);
else
    identityPath = string(java.io.File(char(modelFile)).getCanonicalPath());
end

identityText = join([identityPath, string(blockPath), upper(string(transportName)), ...
    string(targetAddress), string(transmitPort), string(receivePort), ...
    string(requestBytes), string(responseBytes)], "|");
digest = java.security.MessageDigest.getInstance('SHA-256');
hashBytes = typecast(digest.digest(uint8(char(identityText))), 'uint8');
connectionIdBytes = reshape(hashBytes(1:16), 1, []);
connectionIdText = lower(reshape(dec2hex(connectionIdBytes, 2).', 1, []));

% Level-2 C S-functions receive ordinary numeric mask parameters through
% mxGetPr.  Keep this public parameter a double vector and narrow each byte
% explicitly inside the MEX file; returning uint8 here would make the legacy
% generic parameter accessor interpret a byte array as double storage.
ConnectionId = double(connectionIdBytes);

% Opening the source library must not create a controller configuration.
if bdIsLibrary(rootModel) || strlength(modelFile) == 0
    return;
end

config = struct( ...
    'schema_version', 1, ...
    'protocol_version', 1, ...
    'transport', lower(char(string(transportName))), ...
    'role', 'server', ...
    'connection_id', connectionIdText, ...
    'target_address', char(string(targetAddress)), ...
    'bind_address', '0.0.0.0', ...
    'transmit_port', double(receivePort), ...
    'receive_port', double(transmitPort), ...
    'simulink_to_controller_bytes', double(requestBytes), ...
    'controller_to_simulink_bytes', double(responseBytes), ...
    'connect_timeout_ms', 5000, ...
    'startup_io_timeout_ms', 5000, ...
    'startup_timeout_enabled', false, ...
    'established_io_timeout_ms', 2000000, ...
    'established_after_frames', 10, ...
    'max_payload', max([double(requestBytes), double(responseBytes), 65536]));

modelDirectory = fileparts(modelFile);
configPath = fullfile(modelDirectory, 'network.json');
temporaryPath = [tempname(modelDirectory), '.json'];
fileId = fopen(temporaryPath, 'w', 'n', 'UTF-8');
if fileId < 0
    error("GMP:SIL:ConfigWriteFailed", "Cannot create temporary SIL configuration file.");
end
cleanup = onCleanup(@() cleanupTemporary(temporaryPath));
try
    fprintf(fileId, '%s\n', jsonencode(config, PrettyPrint=true));
    fclose(fileId);
catch writeError
    try %#ok<TRYNC>
        fclose(fileId);
    end
    rethrow(writeError);
end
[moved, message] = movefile(temporaryPath, configPath, 'f');
if ~moved
    error("GMP:SIL:ConfigWriteFailed", "Cannot publish network.json: %s", message);
end
clear cleanup;
end

function cleanupTemporary(temporaryPath)
if isfile(temporaryPath)
    delete(temporaryPath);
end
end
