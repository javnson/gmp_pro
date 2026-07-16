function y = ctl_atan2(Y, X)
%CTL_ATAN2 Four-quadrant arc tangent in radians, matching atan2f().
y = single(atan2(single(Y), single(X)));
end
