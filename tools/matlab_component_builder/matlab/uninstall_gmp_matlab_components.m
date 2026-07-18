% Uninstall paths registered by the GMP MATLAB Component Builder.
toolRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(toolRoot, 'matlab'));
gmp_mcb.uninstall();

