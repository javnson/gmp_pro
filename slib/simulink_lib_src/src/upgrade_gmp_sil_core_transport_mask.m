function upgrade_gmp_sil_core_transport_mask(modelFile)
%UPGRADE_GMP_SIL_CORE_TRANSPORT_MASK Upgrade SIL blocks and export R2022b.
% The public export workflow passes a temporary copy of the editable R2024b
% library. This helper never needs a second maintained C++ source tree.

arguments
    modelFile {mustBeFile}
end

load_system(modelFile);
model = bdroot;
cleanup = onCleanup(@() close_system(model, 0));
wasLocked = strcmp(get_param(model, 'Lock'), 'on');
if wasLocked
    set_param(model, 'Lock', 'off');
end
blocks = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
    'MaskType', 'GMP CTL SIL Core');
blocks = blocks(strcmp(get_param(blocks, 'LinkStatus'), 'none'));

initialization = [ ...
    "[NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes, ...", newline, ...
    " NumberPackedPorts, PackedDataTypes, PackedDataSizes, ...", newline, ...
    " Alignment, PackedBytes, UnpackedBytes] = mdl_gmp_simulink_core( ...", newline, ...
    " MaskUnpackedDataTypes, MaskUnpackedDataSizes, ...", newline, ...
    " MaskPackedDataTypes, MaskPackedDataSizes, MaskAlignment);", newline, newline, ...
    "[Transport, HostAddr, TargetPort] = mdl_gmp_simulink_connection( ...", newline, ...
    " MaskTransport, MaskHostAddr, MaskMsgTxPort, MaskMsgRxPort);", newline, newline, ...
    "ConnectionId = gmp_prepare_sil_connection(gcb, MaskTransport, ...", newline, ...
    " MaskHostAddr, MaskMsgTxPort, MaskMsgRxPort, PackedBytes, UnpackedBytes);" ...
    ];
parameterExpression = [ ...
    "NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes, ", ...
    "NumberPackedPorts, PackedDataTypes, PackedDataSizes, Alignment, ", ...
    "Transport, HostAddr, TargetPort, ConnectionId" ...
    ];

for index = 1:numel(blocks)
    block = blocks{index};
    mask = Simulink.Mask.get(block);
    removeIfPresent(mask, 'MaskCmdTxPort');
    removeIfPresent(mask, 'MaskCmdRxPort');
    removeDialogIfPresent(mask, 'Control11');
    removeDialogIfPresent(mask, 'Control12');
    legacyCommandPanel = mask.getDialogControl('Container7');
    if ~isempty(legacyCommandPanel)
        legacyCommandPanel.Visible = 'off';
        legacyCommandPanel.Prompt = '';
    end

    transportParameter = mask.getParameter('MaskTransport');
    if isempty(transportParameter)
        transportParameter = mask.addParameter( ...
            'Type', 'popup', 'TypeOptions', {'UDP', 'TCP'}, ...
            'Name', 'MaskTransport', 'Prompt', 'Transport', 'Value', 'UDP');
    end
    transportParameter.Type = 'popup';
    transportParameter.TypeOptions = {'UDP', 'TCP'};
    transportParameter.Prompt = 'Transport';
    transportParameter.Value = 'UDP';
    transportParameter.Evaluate = 'off';
    transportParameter.Tunable = 'off';
    transportParameter.Callback = 'gmp_update_sil_transport_mask(gcb);';

    setPrompt(mask, 'MaskHostAddr', 'Target address');
    setPrompt(mask, 'MaskMsgTxPort', 'Target receive port');
    setPrompt(mask, 'MaskMsgRxPort', 'Local receive port (UDP)');
    mask.Initialization = char(join(initialization, ''));
    displayCode = [ ...
        "image(append(fileparts(which('gmp_simulink_utilities.slx')),'\icon\GMP_Core.png'));", newline, ...
        "color('blue');", newline, ...
        "disp(['GMP SIL CORE Module\n', get_param(gcb,'MaskTransport')]);", newline, ...
        "color('Black');" ...
        ];
    mask.Display = char(join(displayCode, ''));
    set_param(block, 'Parameters', char(join(parameterExpression, '')));
    % The library distributes the MATLAB-release-compatible MEX artifact.
    % Do not request a same-name .c source module from generated targets.
    set_param(block, 'SFunctionModules', '');
end

if wasLocked
    set_param(model, 'Lock', 'on');
end
if strcmpi(version('-release'), '2022b')
    save_system(model, modelFile);
    clear cleanup;
    close_system(model, 0);
else
    exportPath = [tempname(fileparts(modelFile)), '.slx'];
    exportCleanup = onCleanup(@() deleteIfPresent(exportPath));
    Simulink.exportToVersion(model, exportPath, 'R2022B', AllowPrompt=false);
    clear cleanup;
    close_system(model, 0);
    [moved, message] = movefile(exportPath, modelFile, 'f');
    if ~moved
        error("GMP:SIL:ModelExportFailed", "Cannot publish R2022b source library: %s", message);
    end
    clear exportCleanup;
end
end

function deleteIfPresent(path)
if isfile(path)
    delete(path);
end
end

function removeIfPresent(mask, parameterName)
if ~isempty(mask.getParameter(parameterName))
    mask.removeParameter(parameterName);
end
end

function removeDialogIfPresent(mask, controlName)
if ~isempty(mask.getDialogControl(controlName))
    mask.removeDialogControl(controlName);
end
end

function setPrompt(mask, parameterName, prompt)
parameter = mask.getParameter(parameterName);
if ~isempty(parameter)
    parameter.Prompt = prompt;
end
end
