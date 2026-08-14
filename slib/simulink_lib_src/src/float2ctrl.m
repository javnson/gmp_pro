function y = float2ctrl(x)
%FLOAT2CTRL Deprecated compatibility wrapper for real2ctrl.
warning('GMP:DeprecatedNumericConversion', ...
    'float2ctrl is deprecated; use real2ctrl or param2ctrl.');
y = real2ctrl(x);
end
