function y = ctrl2int(x)
%CTRL2INT Convert ctrl_gt to C int semantics (truncate toward zero).
y = int32(fix(single(x)));
end
