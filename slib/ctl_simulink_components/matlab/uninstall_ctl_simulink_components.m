% Uninstall paths registered by CTL Simulink Components.
toolRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(toolRoot, 'matlab'));
gmp_mcb.uninstall();
