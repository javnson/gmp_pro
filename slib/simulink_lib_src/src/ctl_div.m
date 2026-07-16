function y = ctl_div(A, B)
%CTL_DIV Divide float-backed ctrl_gt values.
y = single(single(A) ./ single(B));
end
