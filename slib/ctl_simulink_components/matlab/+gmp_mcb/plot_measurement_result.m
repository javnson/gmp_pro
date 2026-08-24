function figureHandle = plot_measurement_result(result, blockName)
% Plot a measured response, optionally overlaid with analytical references.
if nargin < 2 || strlength(string(blockName)) == 0
    blockName = result.componentName;
end

figureHandle = figure('Name', ['Measured GMP Component: ' char(blockName)]);
layout = tiledlayout(figureHandle, 2, 1);
title(layout, sprintf('%s | %s to %s | excitation %.6g | bias %.6g | execution %.6g Hz', ...
    char(result.componentName), char(result.inputLabel), char(result.outputLabel), ...
    result.amplitude, result.bias, result.executionFs));

nexttile;
if result.hasReferenceModel
    semilogx(result.frequencyHz, 20 * log10(abs(result.continuous)), 'LineWidth', 1.2); hold on;
    semilogx(result.frequencyHz, 20 * log10(abs(result.implementation)), '--', 'LineWidth', 1.2);
    semilogx(result.frequencyHz, 20 * log10(abs(result.measured)), 'o', 'LineWidth', 1.2);
    legend('Ideal continuous', 'Implementation reference', 'Measured MEX', 'Location', 'best');
else
    semilogx(result.frequencyHz, 20 * log10(abs(result.measured)), 'o-', 'LineWidth', 1.2);
    legend('Measured MEX', 'Location', 'best');
end
grid on; ylabel('Magnitude [dB]');

nexttile;
if result.hasReferenceModel
    semilogx(result.frequencyHz, unwrap(angle(result.continuous)) * 180 / pi, 'LineWidth', 1.2); hold on;
    semilogx(result.frequencyHz, unwrap(angle(result.implementation)) * 180 / pi, '--', 'LineWidth', 1.2);
    semilogx(result.frequencyHz, unwrap(angle(result.measured)) * 180 / pi, 'o', 'LineWidth', 1.2);
else
    semilogx(result.frequencyHz, unwrap(angle(result.measured)) * 180 / pi, 'o-', 'LineWidth', 1.2);
end
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
end
