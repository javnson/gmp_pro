function y = ctl_abs(A)
%CTL_ABS Absolute value with float_macros.h comparison semantics.
y = abs_static_inline(A);
end
