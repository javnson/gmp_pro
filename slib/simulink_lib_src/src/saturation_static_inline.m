function y = saturation_static_inline(A, max_val, min_val)
%SATURATION_STATIC_INLINE MATLAB equivalent of the float backend helper.
A = single(A); max_val = single(max_val); min_val = single(min_val);
y = A;
below = A < min_val;
above = A > max_val;
if isscalar(min_val), y(below) = min_val; else, y(below) = min_val(below); end
if isscalar(max_val), y(above) = max_val; else, y(above) = max_val(above); end
end
