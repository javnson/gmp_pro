function tests = test_sil_connection
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
sourceDirectory = fullfile(getenv('GMP_PRO_LOCATION'), 'slib', 'simulink_lib_src', 'src');
testCase.TestData.OriginalPath = path;
addpath(sourceDirectory);
end

function teardownOnce(testCase)
path(testCase.TestData.OriginalPath);
end

function testLocalhostUdp(testCase)
lastwarn('');
[transport, host, ports] = mdl_gmp_simulink_connection('UDP', 'localhost', 12300, 12301);
verifyEqual(testCase, transport, 0);
verifyEqual(testCase, host, [127, 0, 0, 1]);
verifyEqual(testCase, ports, [12300, 12301]);
verifyEmpty(testCase, lastwarn);
end

function testTransportRecommendations(testCase)
lastwarn('');
mdl_gmp_simulink_connection('TCP', '127.0.0.1', 12300, 12301);
[~, identifier] = lastwarn;
verifyEqual(testCase, identifier, 'GMP:SIL:PreferUDP');

lastwarn('');
mdl_gmp_simulink_connection('UDP', '192.168.0.9', 12300, 12301);
[~, identifier] = lastwarn;
verifyEqual(testCase, identifier, 'GMP:SIL:PreferTCP');

lastwarn('');
mdl_gmp_simulink_connection('TCP', '192.168.0.9', 12300, 12301);
verifyEmpty(testCase, lastwarn);
end

function testInvalidConnectionInput(testCase)
verifyError(testCase, @() mdl_gmp_simulink_connection('SCTP', '127.0.0.1', 1, 2), ...
    'GMP:SIL:InvalidTransport');
verifyError(testCase, @() mdl_gmp_simulink_connection('UDP', '127.0.0.1.bad', 1, 2), ...
    'GMP:SIL:InvalidAddress');
verifyError(testCase, @() mdl_gmp_simulink_connection('UDP', '127.0.0.1', 12300, 12300), ...
    'GMP:SIL:DuplicateUdpPort');
end

function testSessionJsonContainsIdentityAndAbi(testCase)
temporaryDirectory = tempname;
mkdir(temporaryDirectory);
cleanupDirectory = onCleanup(@() rmdir(temporaryDirectory, 's'));
model = 'gmp_sil_config_test_model';
new_system(model);
cleanupModel = onCleanup(@() close_system(model, 0));
modelFile = fullfile(temporaryDirectory, [model, '.slx']);
save_system(model, modelFile);

firstId = gmp_prepare_sil_connection(model, 'TCP', '192.168.0.9', 12500, 12501, 264, 200);
secondId = gmp_prepare_sil_connection(model, 'TCP', '192.168.0.9', 12500, 12501, 264, 200);
verifyEqual(testCase, firstId, secondId);
verifySize(testCase, firstId, [1, 16]);

document = jsondecode(fileread(fullfile(temporaryDirectory, 'network.json')));
verifyEqual(testCase, document.schema_version, 1);
verifyEqual(testCase, document.protocol_version, 1);
verifyEqual(testCase, document.transport, 'tcp');
verifyEqual(testCase, document.role, 'server');
verifyEqual(testCase, document.receive_port, 12500);
verifyEqual(testCase, document.transmit_port, 12501);
verifyEqual(testCase, document.simulink_to_controller_bytes, 264);
verifyEqual(testCase, document.controller_to_simulink_bytes, 200);
verifyEqual(testCase, numel(document.connection_id), 32);
verifyFalse(testCase, document.startup_timeout_enabled);
verifyEqual(testCase, document.established_after_frames, 10);

close_system(model, 0);
end
