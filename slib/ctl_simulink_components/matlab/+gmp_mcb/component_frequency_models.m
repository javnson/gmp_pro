function models = component_frequency_models(block, frequenciesHz)
% Dispatch frequency-model construction by generated component template.
component = get_param(block, 'UserData');
if ~isstruct(component) || ~isfield(component, 'template')
    error('GMP:MCB:Metadata', 'Block does not contain generated component metadata.');
end
switch char(component.template)
    case 'pid_siso_v2'
        models = gmp_mcb.pid_frequency_models(block, frequenciesHz);
        models.hasReferenceModel = true;
    case 'resonant_siso_v1'
        models = gmp_mcb.resonant_frequency_models(block, frequenciesHz);
        models.hasReferenceModel = true;
    case 'generic_stateful_v1'
        frequenciesHz = frequenciesHz(:);
        models = struct('frequencyHz', frequenciesHz, ...
            'continuous', complex(nan(size(frequenciesHz))), ...
            'implementation', complex(nan(size(frequenciesHz))), ...
            'hasReferenceModel', false, ...
            'modeLabel', "Measured-only component", ...
            'parameterFs', optional_mask_value(block, 'fs', NaN), ...
            'executionFs', gmp_mcb.mask_value(block, 'analysis_execution_fs'));
    otherwise
        error('GMP:MCB:Template', 'No frequency model for template %s.', component.template);
end
end

function value = optional_mask_value(block, name, defaultValue)
mask = Simulink.Mask.get(block);
if isempty(mask) || isempty(mask.getParameter(name))
    value = defaultValue;
else
    value = gmp_mcb.mask_value(block, name);
end
end
