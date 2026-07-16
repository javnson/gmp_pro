function y = ctl_mul4(A)
%CTL_MUL4 Multiply a float-backed ctrl_gt value by four.
y = single(single(A) .* single(4));
end
