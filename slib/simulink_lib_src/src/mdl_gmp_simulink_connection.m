

% Test Command
% [Transport, HostAddr, TargetPort] = ...
%     mdl_gmp_simulink_connection("UDP", "127.0.0.1", 12300, 12301)


% HostAddr =
%    127     0     0     1


% TargetPort =
%        12300       12301       12302       12303

function [Transport, HostAddr, TargetPort] = mdl_gmp_simulink_connection( ...
    MaskTransport, MaskHostAddr, MaskMsgTxPort, MaskMsgRxPort)

transportName = upper(string(MaskTransport));
if transportName == "UDP"
    Transport = 0;
elseif transportName == "TCP"
    Transport = 1;
else
    error("GMP:SIL:InvalidTransport", ...
        "Transport must be selected as TCP or UDP.");
end

hostName = lower(strtrim(string(MaskHostAddr)));
if hostName == "localhost"
    hostName = "127.0.0.1";
end

% validate IP address
IP_REGEX = "^(((\d)|([1-9]\d)|(1\d{2})|(2[0-4]\d)|(25[0-5]))\.){3}((\d)|([1-9]\d)|(1\d{2})|(2[0-4]\d)|(25[0-5]))$";
if isempty(regexp(hostName, IP_REGEX, 'once'))
    error("GMP:SIL:InvalidAddress", ...
        "Target address must be localhost or a numeric IPv4 address.");
end

splitAddr = split(hostName,".");
HostAddr = str2double(splitAddr)';

ports = double([MaskMsgTxPort, MaskMsgRxPort]);
if any(~isfinite(ports)) || any(ports < 1) || any(ports > 65535) || any(fix(ports) ~= ports)
    error("GMP:SIL:InvalidPort", "SIL ports must be integer values in [1, 65535].");
end
if ports(1) == ports(2) && Transport == 0
    error("GMP:SIL:DuplicateUdpPort", "UDP transmit and receive ports must be different.");
end
TargetPort = ports;

isLocal = isequal(HostAddr, [127, 0, 0, 1]);
if isLocal && Transport == 1
    warning("GMP:SIL:PreferUDP", ...
        "The SIL target is local. UDP is recommended for lower simulation overhead.");
elseif ~isLocal && Transport == 0
    warning("GMP:SIL:PreferTCP", ...
        "The SIL target is remote. TCP is recommended to preserve the complete byte stream.");
end

end
