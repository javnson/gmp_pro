function metrics = sil_step_metrics(response, reference, step_time, tolerance)
%SIL_STEP_METRICS Extract deterministic 10-90%, overshoot and settling data.
%   RESPONSE is a Simulink timeseries. REFERENCE may be a scalar or another
%   timeseries. STEP_TIME is normally the controller-enable or load-step edge.
arguments
    response
    reference
    step_time (1,1) double {mustBeNonnegative} = 0
    tolerance (1,1) double {mustBePositive} = 0.02
end

t = double(response.Time(:));
y = squeeze(double(response.Data)); y = y(:);
valid = isfinite(t) & isfinite(y) & t >= step_time;
t = t(valid); y = y(valid);
if isa(reference, 'timeseries')
    r = interp1(double(reference.Time(:)), double(reference.Data(:)), t, ...
        'previous', 'extrap');
else
    r = repmat(double(reference), size(t));
end
metrics = empty_metrics(step_time);
if numel(y) < 3, return; end

tail_count = max(1, floor(0.2*numel(y)));
initial_count = max(1, floor(0.02*numel(y)));
y0 = mean(y(1:initial_count));
target = mean(r(end-tail_count+1:end));
final = mean(y(end-tail_count+1:end));
amplitude = target-y0;
metrics.initial_value = y0;
metrics.reference_final = target;
metrics.response_final = final;
metrics.steady_state_error_abs = abs(target-final);
metrics.steady_state_error_percent = 100*abs(target-final)/max(abs(target), eps);
if abs(amplitude) <= 100*eps(max(abs([y0 target])))
    return;
end

direction = sign(amplitude);
progress = direction*(y-y0);
level10 = 0.1*abs(amplitude);
level90 = 0.9*abs(amplitude);
i10 = find(progress >= level10, 1, 'first');
i90 = find(progress >= level90, 1, 'first');
if ~isempty(i10) && ~isempty(i90) && i90 >= i10
    metrics.rise_time_10_90_s = t(i90)-t(i10);
end
if direction > 0
    metrics.overshoot_percent = 100*max(0, max(y)-target)/max(abs(amplitude),eps);
else
    metrics.overshoot_percent = 100*max(0, target-min(y))/max(abs(amplitude),eps);
end
band = tolerance*max(abs(amplitude), abs(target)*0.05);
outside = find(abs(y-target) > band, 1, 'last');
if isempty(outside)
    metrics.settling_time_s = 0;
elseif outside < numel(t)
    metrics.settling_time_s = t(outside+1)-step_time;
end
metrics.dynamic_valid = isfinite(metrics.rise_time_10_90_s) && ...
    isfinite(metrics.settling_time_s);
end

function metrics = empty_metrics(step_time)
metrics = struct('step_time_s', step_time, 'initial_value', NaN, ...
    'reference_final', NaN, 'response_final', NaN, ...
    'rise_time_10_90_s', NaN, 'settling_time_s', NaN, ...
    'overshoot_percent', NaN, 'steady_state_error_abs', NaN, ...
    'steady_state_error_percent', NaN, 'dynamic_valid', false);
end
