function y = ctl_sin(A)
%CTL_SIN Sine of a per-unit angle (1 pu equals 2*pi radians).
y = single(sin(single(2*pi) .* single(A)));
end
