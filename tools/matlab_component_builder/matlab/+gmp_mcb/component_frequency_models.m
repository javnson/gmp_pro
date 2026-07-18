function models = component_frequency_models(block, frequenciesHz)
% Dispatch frequency-model construction by generated component template.
component = get_param(block, 'UserData');
if ~isstruct(component) || ~isfield(component, 'template')
    error('GMP:MCB:Metadata', 'Block does not contain generated component metadata.');
end
switch char(component.template)
    case 'pid_siso_v2'
        models = gmp_mcb.pid_frequency_models(block, frequenciesHz);
    case 'resonant_siso_v1'
        models = gmp_mcb.resonant_frequency_models(block, frequenciesHz);
    otherwise
        error('GMP:MCB:Template', 'No frequency model for template %s.', component.template);
end
end

