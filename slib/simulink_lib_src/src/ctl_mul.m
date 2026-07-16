function y = ctl_mul(A, B)
%CTL_MUL Multiply float-backed ctrl_gt values.
y = single(single(A) .* single(B));
end
