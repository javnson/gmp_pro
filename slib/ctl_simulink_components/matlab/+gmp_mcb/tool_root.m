function root = tool_root()
% Return this tool's root without relying on MATLAB's current directory.
packageDir = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(packageDir));
end

