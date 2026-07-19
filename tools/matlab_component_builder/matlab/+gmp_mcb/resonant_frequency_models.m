function models = resonant_frequency_models(block, frequenciesHz)
% Evaluate continuous and exact GMP discrete resonant controller responses.
component = get_param(block, 'UserData');
variant = char(component.variant);
parameterFs = gmp_mcb.mask_value(block, 'fs');
executionFs = gmp_mcb.mask_value(block, 'analysis_execution_fs');
kr = gmp_mcb.mask_value(block, 'kr');
fr = gmp_mcb.mask_value(block, 'freq_resonant');
if parameterFs <= 0 || executionFs <= 0 || fr <= 0 || fr >= parameterFs / 2 || kr < 0
    error('GMP:MCB:ResonantParameters', 'Require fs>0, execution frequency>0, 0<f_res<fs/2, and Kr>=0.');
end

frequency = frequenciesHz(:);
s = 1i * 2 * pi * frequency;
q = exp(-s / executionFs);
wr = 2 * pi * fr;
hasProportional = any(strcmp(variant, {'pr', 'qpr'}));
isQuasi = any(strcmp(variant, {'qr', 'qpr'}));
kp = 0;
if hasProportional, kp = gmp_mcb.mask_value(block, 'kp'); end

if ~isQuasi
    continuousResonant = kr * 2 .* s ./ (s .^ 2 + wr ^ 2);
    T = 1 / parameterFs;
    wrSqTSq = wr ^ 2 * T ^ 2;
    den = wrSqTSq + 4;
    b0 = kr * 2 * T / den;
    b2 = -b0;
    a1 = 2 * (4 - wrSqTSq) / den;
    a2 = -1;
    discreteResonant = (b0 + b2 .* q .^ 2) ./ (1 - a1 .* q - a2 .* q .^ 2);
else
    fc = gmp_mcb.mask_value(block, 'freq_cut');
    if fc < 0, error('GMP:MCB:Bandwidth', 'Resonant bandwidth must be nonnegative.'); end
    wc = 2 * pi * fc;
    continuousResonant = kr * 2 * wc .* s ./ (s .^ 2 + 2 * wc .* s + wr ^ 2);
    mode = gmp_mcb.init_mode_code(get_param(block, 'init_method'));
    if mode == 1
        kTustin = 2 * parameterFs;
        modeLabel = 'Standard Tustin';
    else
        halfAngle = min(max(pi * fr / parameterFs, 1e-6), pi / 2 - 1e-6);
        kTustin = wr / tan(halfAngle);
        modeLabel = 'Prewarped Tustin';
    end
    D0 = kTustin ^ 2 + 2 * wc * kTustin + wr ^ 2;
    b0 = 2 * kr * wc * kTustin / D0;
    b2 = -b0;
    a1 = (2 * kTustin ^ 2 - 2 * wr ^ 2) / D0;
    a2 = (2 * wc * kTustin - kTustin ^ 2 - wr ^ 2) / D0;
    discreteResonant = (b0 + b2 .* q .^ 2) ./ (1 - a1 .* q - a2 .* q .^ 2);
end
if ~exist('modeLabel', 'var'), modeLabel = 'Standard Tustin'; end
models = struct('frequencyHz', frequency, ...
    'continuous', kp + continuousResonant, ...
    'implementation', kp + discreteResonant, ...
    'modeLabel', upper(variant) + " / " + modeLabel, ...
    'parameterFs', parameterFs, 'executionFs', executionFs);
end

