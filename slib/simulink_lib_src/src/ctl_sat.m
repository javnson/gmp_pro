function y = ctl_sat(A, Pos, Neg)
%CTL_SAT Saturate A between Neg and Pos (argument order matches GMP C).
y = saturation_static_inline(A, Pos, Neg);
end
