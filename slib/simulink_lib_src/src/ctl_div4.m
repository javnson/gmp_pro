function y = ctl_div4(A)
%CTL_DIV4 Divide a float-backed ctrl_gt value by four.
y = single(single(A) ./ single(4));
end
