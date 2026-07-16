function y = ctl_cos(A)
%CTL_COS Cosine of a per-unit angle (1 pu equals 2*pi radians).
y = single(cos(single(2*pi) .* single(A)));
end
