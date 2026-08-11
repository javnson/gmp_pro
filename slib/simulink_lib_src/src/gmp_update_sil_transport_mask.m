function gmp_update_sil_transport_mask(blockPath)
%GMP_UPDATE_SIL_TRANSPORT_MASK Show only transport-relevant network fields.
% TCP uses one connected socket and therefore does not need a separate local
% receive port. UDP keeps distinct target and local receive ports.

arguments
    blockPath {mustBeTextScalar}
end

names = get_param(blockPath, 'MaskNames');
visibilities = get_param(blockPath, 'MaskVisibilities');
receivePortIndex = find(strcmp(names, 'MaskMsgRxPort'), 1);
if isempty(receivePortIndex)
    return;
end

transport = upper(string(get_param(blockPath, 'MaskTransport')));
if transport == "TCP"
    visibilities{receivePortIndex} = 'off';
else
    visibilities{receivePortIndex} = 'on';
end
set_param(blockPath, 'MaskVisibilities', visibilities);
end
