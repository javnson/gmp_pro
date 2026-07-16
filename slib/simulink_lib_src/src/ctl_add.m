function y = ctl_add(A, B)
%CTL_ADD Add float-backed ctrl_gt values.
y = single(single(A) + single(B));
end
