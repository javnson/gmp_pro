function y = ctl_div2(A)
%CTL_DIV2 Divide a float-backed ctrl_gt value by two.
y = single(single(A) ./ single(2));
end
