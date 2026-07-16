function y = ctl_tan(A)
%CTL_TAN Tangent of a per-unit angle (1 pu equals 2*pi radians).
y = single(tan(single(2*pi) .* single(A)));
end
