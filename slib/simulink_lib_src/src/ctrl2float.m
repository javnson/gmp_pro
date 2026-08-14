function y = ctrl2float(x)
%CTRL2FLOAT Deprecated compatibility wrapper for ctrl2param.
warning('GMP:DeprecatedNumericConversion', ...
    'ctrl2float is deprecated; use ctrl2param.');
y = single(ctrl2param(x));
end
