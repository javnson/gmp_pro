function value = mask_value(block, name)
% Resolve a numeric mask expression in the block/model workspace.
expression = get_param(block, name);
try
    value = slResolve(expression, block);
catch
    value = str2double(expression);
end
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error('GMP:MCB:MaskValue', 'Mask parameter %s must resolve to a finite numeric scalar.', name);
end
value = double(value);
end

