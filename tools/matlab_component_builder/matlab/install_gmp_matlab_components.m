% Install the independent GMP MATLAB Component Builder library.
toolRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(toolRoot, 'matlab'));
gmp_mcb.install();

