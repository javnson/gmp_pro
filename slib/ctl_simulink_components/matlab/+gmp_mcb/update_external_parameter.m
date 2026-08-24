function update_external_parameter(block, parameterName)
% Disable the fixed-value editor when its external input port is selected.
mask = Simulink.Mask.get(block);
expose = get_param(block, ['expose_' parameterName]);
parameter = mask.getParameter(parameterName);
if gmp_mcb.checkbox_code(expose)
    parameter.Enabled = 'off';
else
    parameter.Enabled = 'on';
end
end

