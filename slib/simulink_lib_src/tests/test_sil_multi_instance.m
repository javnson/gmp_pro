function tests = test_sil_multi_instance
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = string(getenv('GMP_PRO_LOCATION'));
testCase.TestData.Source = fullfile(root, 'slib', 'simulink_lib_src', 'src');
testCase.TestData.Peer = fullfile(root, 'tools', 'gmp_sil', 'sil_helper', ...
    'build', 'tests', 'Release', 'gmp_sil_test_peer.exe');
testCase.TestData.OriginalPath = path;
addpath(testCase.TestData.Source);
mexOverride = string(getenv('GMP_SIL_MEX_OVERRIDE'));
if strlength(mexOverride) > 0 && isfile(fullfile(mexOverride, ['GMP_SIL_Core.' mexext]))
    addpath(mexOverride, '-begin');
end
end

function teardownOnce(testCase)
path(testCase.TestData.OriginalPath);
end

function testTwoUdpCoresInOneModel(testCase)
assumeTrue(testCase, ispc && isfile(testCase.TestData.Peer), ...
    'Build gmp_sil_test_peer before running the Windows integration test.');

temporaryDirectory = tempname;
mkdir(temporaryDirectory);
cleanupDirectory = onCleanup(@() rmdir(temporaryDirectory, 's'));
ports = uniqueUdpPorts(4);
ids = [uint8(0:15); uint8(160:175)];
configs = strings(1, 2);
for index = 1:2
    configs(index) = fullfile(temporaryDirectory, "peer_" + index + ".json");
    writeConfig(configs(index), ports(index * 2 - 1), ports(index * 2), ids(index, :));
end

peers = {startPeer(testCase.TestData.Peer, configs(1)), ...
         startPeer(testCase.TestData.Peer, configs(2))};
cleanupPeers = onCleanup(@() stopPeers(peers));
pause(0.1); % Controller/server startup precedes the Simulink clients.

model = 'gmp_sil_two_core_test';
new_system(model);
cleanupModel = onCleanup(@() close_system(model, 0));
for index = 1:2
    constant = sprintf('%s/input_%d', model, index);
    core = sprintf('%s/core_%d', model, index);
    sink = sprintf('%s/output_%d', model, index);
    y = 40 + (index - 1) * 100;
    add_block('simulink/Sources/Constant', constant, ...
        'Value', sprintf('uint8(%d:%d)', index, index + 31), ...
        'OutDataTypeStr', 'uint8', 'SampleTime', '1e-4', ...
        'Position', [30 y 100 y + 30]);
    add_block('simulink/User-Defined Functions/S-Function', core, ...
        'FunctionName', 'GMP_SIL_Core', ...
        'Parameters', parameterText(ports(index * 2 - 1), ports(index * 2), ids(index, :)), ...
        'Position', [150 y 280 y + 30]);
    add_block('simulink/Sinks/Terminator', sink, ...
        'Position', [330 y 350 y + 30]);
    add_line(model, sprintf('input_%d/1', index), sprintf('core_%d/1', index));
    add_line(model, sprintf('core_%d/1', index), sprintf('output_%d/1', index));
end
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1e-4', 'StopTime', '4e-4');
sim(model, 'SimulationMode', 'normal');

for index = 1:numel(peers)
    verifyTrue(testCase, peers{index}.WaitForExit(5000), ...
        sprintf('SIL peer %d did not receive explicit completion.', index));
    verifyEqual(testCase, peers{index}.ExitCode, int32(0));
end

clear cleanupModel cleanupPeers cleanupDirectory;
end

function testTwoModelsInSeparateMatlabProcesses(testCase)
assumeTrue(testCase, ispc && isfile(testCase.TestData.Peer), ...
    'Build gmp_sil_test_peer before running the Windows integration test.');

temporaryDirectory = tempname;
mkdir(temporaryDirectory);
cleanupDirectory = onCleanup(@() rmdir(temporaryDirectory, 's'));
ports = uniqueUdpPorts(4);
ids = [uint8(32:47); uint8(208:223)];
configs = strings(1, 2);
for index = 1:2
    configs(index) = fullfile(temporaryDirectory, "model_peer_" + index + ".json");
    writeConfig(configs(index), ports(index * 2 - 1), ports(index * 2), ids(index, :));
end

peers = {startPeer(testCase.TestData.Peer, configs(1)), ...
         startPeer(testCase.TestData.Peer, configs(2))};
cleanupPeers = onCleanup(@() stopPeers(peers));
pause(0.1);

matlabs = cell(1, 2);
for index = 1:2
    idText = lower(reshape(dec2hex(ids(index, :), 2).', 1, []));
    command = sprintf(["setenv('GMP_PRO_LOCATION','%s');" + ...
        "addpath('%s');addpath('%s');" + ...
        "run_sil_single_model_child(%d,%d,'%s',%d);"], ...
        strrep(char(getenv('GMP_PRO_LOCATION')), '\', '/'), ...
        strrep(char(testCase.TestData.Source), '\', '/'), ...
        strrep(fileparts(mfilename('fullpath')), '\', '/'), ...
        ports(index * 2 - 1), ports(index * 2), idText, index);
    matlabs{index} = startMatlab(command);
end
cleanupMatlabs = onCleanup(@() stopProcesses(matlabs));

for index = 1:numel(matlabs)
    verifyTrue(testCase, matlabs{index}.WaitForExit(120000), ...
        sprintf('MATLAB child %d did not finish its independent model.', index));
    verifyEqual(testCase, matlabs{index}.ExitCode, int32(0));
end
for index = 1:numel(peers)
    verifyTrue(testCase, peers{index}.WaitForExit(5000));
    verifyEqual(testCase, peers{index}.ExitCode, int32(0));
end

clear cleanupMatlabs cleanupPeers cleanupDirectory;
end

function text = parameterText(serverPort, clientPort, id)
text = sprintf(['1, 3, [1 24], 1, 3, [1 32], 1, 0, ', ...
    '[127 0 0 1], [%d %d], %s'], serverPort, clientPort, mat2str(double(id)));
end

function ports = uniqueUdpPorts(count)
ports = zeros(1, count);
for index = 1:count
    while true
        socket = java.net.DatagramSocket(0);
        candidate = double(socket.getLocalPort());
        socket.close();
        if ~ismember(candidate, ports)
            ports(index) = candidate;
            break;
        end
    end
end
end

function writeConfig(path, serverPort, clientPort, id)
document = struct( ...
    'schema_version', 1, 'protocol_version', 1, 'transport', 'udp', ...
    'role', 'server', 'connection_id', lower(reshape(dec2hex(id, 2).', 1, [])), ...
    'target_address', '127.0.0.1', 'bind_address', '127.0.0.1', ...
    'transmit_port', clientPort, 'receive_port', serverPort, ...
    'simulink_to_controller_bytes', 32, 'controller_to_simulink_bytes', 24, ...
    'connect_timeout_ms', 2000, 'startup_io_timeout_ms', 2000, ...
    'startup_timeout_enabled', false, 'established_io_timeout_ms', 10000, ...
    'established_after_frames', 10, 'max_payload', 4096);
file = fopen(path, 'w', 'n', 'UTF-8');
assert(file >= 0, 'Cannot create SIL integration configuration.');
cleanup = onCleanup(@() fclose(file));
fprintf(file, '%s\n', jsonencode(document, PrettyPrint=true));
clear cleanup;
end

function process = startPeer(executable, config)
info = System.Diagnostics.ProcessStartInfo(char(executable), ['"' char(config) '"']);
info.UseShellExecute = false;
info.CreateNoWindow = true;
process = System.Diagnostics.Process.Start(info);
end

function process = startMatlab(command)
executable = fullfile(matlabroot, 'bin', 'matlab.exe');
arguments = ['-batch "' strrep(char(command), '"', '\"') '"'];
info = System.Diagnostics.ProcessStartInfo(char(executable), arguments);
info.UseShellExecute = false;
info.CreateNoWindow = true;
process = System.Diagnostics.Process.Start(info);
end

function stopProcesses(processes)
for index = 1:numel(processes)
    try
        if ~isempty(processes{index}) && ~processes{index}.HasExited
            processes{index}.Kill();
            processes{index}.WaitForExit(2000);
        end
        if ~isempty(processes{index}), processes{index}.Dispose(); end
    catch
    end
end
end

function stopPeers(peers)
for index = 1:numel(peers)
    try
        if ~peers{index}.HasExited
            peers{index}.Kill();
            peers{index}.WaitForExit(2000);
        end
        peers{index}.Dispose();
    catch
    end
end
end
