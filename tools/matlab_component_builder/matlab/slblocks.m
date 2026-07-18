function blkStruct = slblocks
% Register the independent generated GMP component library.
blkStruct.Name = sprintf('GMP MATLAB Components');
blkStruct.OpenFcn = 'open_system(''gmp_mcb_components'')';
Browser.Library = 'gmp_mcb_components';
Browser.Name = 'GMP MATLAB Components';
blkStruct.Browser = Browser;
end

