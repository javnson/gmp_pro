function y = ctrl_mod_1(x)
%CTRL_MOD_1 Fractional part using C truncation, not MATLAB mod().
x = single(x);
y = single(x - fix(x));
end
