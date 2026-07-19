function models = pid_frequency_models(block, frequenciesHz)
% Evaluate ideal continuous and exact implemented discrete PID responses.
kp = gmp_mcb.mask_value(block, 'kp');
integralValue = gmp_mcb.mask_value(block, 'ki_or_ti');
derivativeValue = gmp_mcb.mask_value(block, 'kd_or_td');
parameterFs = gmp_mcb.mask_value(block, 'fs');
executionFs = gmp_mcb.mask_value(block, 'analysis_execution_fs');
if parameterFs <= 0 || executionFs <= 0
    error('GMP:MCB:Fs', 'Controller parameter fs and analysis/execution frequency must be positive.');
end

frequency = frequenciesHz(:);
s = 1i * 2 * pi * frequency;
zInverse = exp(-s / executionFs);
mode = gmp_mcb.init_mode_code(get_param(block, 'init_method'));
if mode == 1
    continuous = kp + integralValue ./ s + derivativeValue .* s;
    implementation = kp + (integralValue / parameterFs) ./ (1 - zInverse) + ...
        (derivativeValue * parameterFs) .* (1 - zInverse);
    label = 'Parallel gains';
else
    ti = integralValue;
    td = derivativeValue;
    if ti <= 0, error('GMP:MCB:Ti', 'Ti must be positive for frequency analysis.'); end
    continuous = kp .* (1 + 1 ./ (ti .* s) + td .* s);
    % This expression follows the current ctl_step_pid_ser implementation,
    % whose derivative term is not multiplied by Kp.
    implementation = kp + (kp / (parameterFs * ti)) ./ (1 - zInverse) + ...
        (parameterFs * td) .* (1 - zInverse);
    label = 'Time constants';
end
models = struct('frequencyHz', frequency, 'continuous', continuous, ...
    'implementation', implementation, 'modeLabel', label, ...
    'parameterFs', parameterFs, 'executionFs', executionFs);
end
