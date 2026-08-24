function code = checkbox_code(value)
% Convert a mask checkbox value to the numeric S-Function ABI.
if isnumeric(value) || islogical(value)
    code = double(value ~= 0);
else
    code = double(any(strcmpi(strtrim(string(value)), ["on", "true", "1", "yes"])));
end
end

