function y = pwm_sat(A, Pos, Neg)
%PWM_SAT Saturate in float and cast to default GMP pwm_gt (uint32).
y = uint32(fix(saturation_static_inline(single(A), single(Pos), single(Neg))));
end
