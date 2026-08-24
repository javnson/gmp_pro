function blkStruct = slblocks
% Register the generated CTL component library.
blkStruct.Name = sprintf('CTL Simulink Components');
blkStruct.OpenFcn = 'open_system(''ctl_simulink_components'')';
Browser.Library = 'ctl_simulink_components';
Browser.Name = 'CTL Simulink Components';
blkStruct.Browser = Browser;
end
