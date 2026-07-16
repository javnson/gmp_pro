function tests = test_float_macros
%TEST_FLOAT_MACROS Regression tests for the MATLAB float_macros API.
tests = functiontests(localfunctions);
end

function testConversionSemantics(testCase)
verifyClass(testCase, float2ctrl(pi), 'single');
verifyClass(testCase, ctrl2float(1), 'single');
verifyEqual(testCase, int2ctrl(int32(-3)), single(-3));
verifyEqual(testCase, ctrl2int(single([-1.9, 1.9])), int32([-1, 1]));
verifyEqual(testCase, ctrl_mod_1(single([-1.25, 1.25])), single([-0.25, 0.25]));
end

function testArithmeticSemantics(testCase)
verifyEqual(testCase, ctl_mul(single(2), single(3)), single(6));
verifyEqual(testCase, ctl_div(single(3), single(2)), single(1.5));
verifyEqual(testCase, ctl_add(single(2), single(3)), single(5));
verifyEqual(testCase, ctl_sub(single(2), single(3)), single(-1));
verifyEqual(testCase, ctl_div2(single(8)), single(4));
verifyEqual(testCase, ctl_div4(single(8)), single(2));
verifyEqual(testCase, ctl_mul2(single(3)), single(6));
verifyEqual(testCase, ctl_mul4(single(3)), single(12));
verifyEqual(testCase, ctl_abs(single([-2, 2])), single([2, 2]));
y = ctl_sat(single([-2, 0, 2, NaN]), single(1), single(-1));
verifyEqual(testCase, y(1:3), single([-1, 0, 1]));
verifyTrue(testCase, isnan(y(4)));
verifyEqual(testCase, pwm_mul(single(0.5), uint32(1000)), uint32(500));
verifyEqual(testCase, pwm_sat(single(1200), uint32(1000), uint32(0)), uint32(1000));
end

function testNonlinearSemantics(testCase)
tol = single(2e-6);
verifyLessThan(testCase, abs(ctl_sin(single(0.25)) - single(1)), tol);
verifyLessThan(testCase, abs(ctl_cos(single(0.5)) + single(1)), tol);
verifyLessThan(testCase, abs(ctl_tan(single(0.125)) - single(1)), tol);
verifyLessThan(testCase, abs(ctl_atan2(single(1), single(0)) - single(pi/2)), tol);
verifyLessThan(testCase, abs(ctl_exp(single(1)) - single(exp(1))), tol);
% The C macro is exp(log(Index)*B), so ctl_pow(3,2) is 2^3.
verifyLessThan(testCase, abs(ctl_pow(single(3), single(2)) - single(8)), tol);
verifyLessThan(testCase, abs(ctl_ln(single(exp(1))) - single(1)), tol);
verifyEqual(testCase, ctl_sqrt(single(9)), single(3));
verifyEqual(testCase, ctl_isqrt(single(4)), single(0.5));
end
