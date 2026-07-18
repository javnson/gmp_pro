function models = plot_pid_models(block, frequenciesHz)
% Plot the ideal continuous PID and exact GMP difference-equation model.
if nargin < 2 || isempty(frequenciesHz)
    executionFs = gmp_mcb.mask_value(block, 'analysis_fs');
    frequenciesHz = logspace(log10(max(executionFs / 10000, 0.1)), log10(0.4 * executionFs), 120);
end
models = gmp_mcb.pid_frequency_models(block, frequenciesHz);

figure('Name', ['GMP PID Models: ' get_param(block, 'Name')]);
tiledlayout(2, 1);
nexttile;
semilogx(models.frequencyHz, 20 * log10(abs(models.continuous)), 'LineWidth', 1.4); hold on;
semilogx(models.frequencyHz, 20 * log10(abs(models.implementation)), '--', 'LineWidth', 1.4);
grid on; ylabel('Magnitude [dB]');
legend('Ideal continuous', 'GMP discrete implementation', 'Location', 'best');
title(sprintf('%s, parameter fs = %.6g Hz, analysis execution = %.6g Hz', ...
    models.modeLabel, models.parameterFs, models.executionFs));
nexttile;
semilogx(models.frequencyHz, unwrap(angle(models.continuous)) * 180 / pi, 'LineWidth', 1.4); hold on;
semilogx(models.frequencyHz, unwrap(angle(models.implementation)) * 180 / pi, '--', 'LineWidth', 1.4);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
end
