function code = init_mode_code(value)
% Convert a mask popup value to the numeric ABI used by the S-Function.
if isnumeric(value)
    code = value;
elseif startsWith(string(value), "Parallel")
    code = 1;
elseif startsWith(string(value), "Time")
    code = 2;
elseif startsWith(string(value), "Standard")
    code = 1;
elseif startsWith(string(value), "Prewarped")
    code = 2;
else
    error('GMP:MCB:InitMode', 'Unknown PID initialization method: %s', string(value));
end
end
