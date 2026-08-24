% Install the generated CTL Simulink Components library.
toolRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(toolRoot, 'matlab'));
gmp_mcb.install();
