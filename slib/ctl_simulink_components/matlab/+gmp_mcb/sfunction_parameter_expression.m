function expression = sfunction_parameter_expression(component)
% Build the ordered fixed-value and exposure-flag parameter expression.
parts = {'gmp_mcb.init_mode_code(init_method)'};
parameters = component.parameters;
for index = 1:numel(parameters)
    parts{end + 1} = char(parameters(index).id); %#ok<AGROW>
end
for index = 1:numel(parameters)
    parameter = parameters(index);
    if isfield(parameter, 'externalizable') && parameter.externalizable
        parts{end + 1} = sprintf('gmp_mcb.checkbox_code(expose_%s)', char(parameter.id)); %#ok<AGROW>
    end
end
expression = strjoin(parts, ',');
end

