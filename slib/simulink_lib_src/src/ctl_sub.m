function y = ctl_sub(A, B)
%CTL_SUB Subtract float-backed ctrl_gt values.
y = single(single(A) - single(B));
end
