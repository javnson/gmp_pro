function models = plot_component_models(block, frequenciesHz)
% Plot ideal continuous and exact GMP difference-equation responses.
if nargin < 2 || isempty(frequenciesHz)
    fmin = gmp_mcb.mask_value(block, 'analysis_frequency_min');
    fmax = gmp_mcb.mask_value(block, 'analysis_frequency_max');
    points = round(gmp_mcb.mask_value(block, 'analysis_points'));
    if fmin <= 0 || fmax <= fmin || points < 2
        error('GMP:MCB:AnalysisRange', 'Analysis requires 0 < fmin < fmax and at least two points.');
    end
    frequenciesHz = logspace(log10(fmin), log10(fmax), points);
end
models = gmp_mcb.component_frequency_models(block, frequenciesHz);

figure('Name', ['GMP Component Models: ' get_param(block, 'Name')]);
tiledlayout(2, 1);
nexttile;
semilogx(models.frequencyHz, 20 * log10(abs(models.continuous)), 'LineWidth', 1.4); hold on;
semilogx(models.frequencyHz, 20 * log10(abs(models.implementation)), '--', 'LineWidth', 1.4);
grid on; ylabel('Magnitude [dB]');
legend('Ideal continuous', 'GMP discrete implementation', 'Location', 'best');
title(sprintf('%s, initializer fs = %.6g Hz, analysis execution = %.6g Hz', ...
    models.modeLabel, models.parameterFs, models.executionFs));
nexttile;
semilogx(models.frequencyHz, unwrap(angle(models.continuous)) * 180 / pi, 'LineWidth', 1.4); hold on;
semilogx(models.frequencyHz, unwrap(angle(models.implementation)) * 180 / pi, '--', 'LineWidth', 1.4);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
end

