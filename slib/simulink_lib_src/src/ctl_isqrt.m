function y = ctl_isqrt(A)
%CTL_ISQRT Single-precision reciprocal square root.
y = single(single(1) ./ sqrt(single(A)));
end
