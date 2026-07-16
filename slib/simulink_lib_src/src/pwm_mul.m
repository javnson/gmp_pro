function y = pwm_mul(A, B)
%PWM_MUL Multiply and cast to the default GMP pwm_gt (uint32).
% Inputs used for PWM counts are expected to be nonnegative, as in C.
y = uint32(fix(single(A) .* single(B)));
end
