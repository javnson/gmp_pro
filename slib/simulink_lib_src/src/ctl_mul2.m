function y = ctl_mul2(A)
%CTL_MUL2 Multiply a float-backed ctrl_gt value by two.
y = single(single(A) .* single(2));
end
