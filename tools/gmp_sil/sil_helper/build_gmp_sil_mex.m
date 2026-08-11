function artifact = build_gmp_sil_mex(varargin)
%BUILD_GMP_SIL_MEX Build the unified TCP/UDP GMP SIL S-function.
%   ARTIFACT = BUILD_GMP_SIL_MEX(...) compiles GMP_SIL_Core using the
%   compiler selected by MATLAB's MEX configuration.  The canonical source
%   remains under tools/gmp_sil; no C++ source is copied into slib.
%
%   Name-value options:
%     RepositoryRoot    GMP repository root (normally auto-detected)
%     DependencyInclude ASIO/nlohmann-json include directory
%     OutputDirectory   Directory that receives GMP_SIL_Core.<mexext>

scriptDirectory = fileparts(mfilename('fullpath'));
defaultRepositoryRoot = fileparts(fileparts(fileparts(scriptDirectory)));

parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'RepositoryRoot', defaultRepositoryRoot, ...
    @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'DependencyInclude', '', ...
    @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'OutputDirectory', ...
    fullfile(scriptDirectory, 'mex_build', mexext), ...
    @(value) ischar(value) || (isstring(value) && isscalar(value)));
parse(parser, varargin{:});

repositoryRoot = char(parser.Results.RepositoryRoot);
dependencyInclude = char(parser.Results.DependencyInclude);
outputDirectory = char(parser.Results.OutputDirectory);
if isempty(dependencyInclude)
    dependencyInclude = defaultDependencyInclude(repositoryRoot);
end

sourceFile = fullfile(scriptDirectory, 'GMP_SIL_Core.cpp');
validateBuildInputs(sourceFile, dependencyInclude);
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

clear('GMP_SIL_Core');
arguments = {'-R2018a', '-DASIO_STANDALONE', ...
    ['-I' repositoryRoot], ['-I' dependencyInclude], ...
    ['-I' fullfile(matlabroot, 'simulink', 'include')], ...
    '-outdir', outputDirectory, '-output', 'GMP_SIL_Core', sourceFile};
if ispc
    arguments = [arguments, ...
        {'COMPFLAGS=$COMPFLAGS /std:c++17 /MT', ...
         'ws2_32.lib', 'mswsock.lib'}];
else
    arguments = [arguments, ...
        {'CXXFLAGS=$CXXFLAGS -std=c++17 -pthread', ...
         'LDFLAGS=$LDFLAGS -pthread'}];
end

fprintf('GMP SIL: compiling unified TCP/UDP MEX with MATLAB mex.\n');
try
    mex(arguments{:});
catch compileError
    error('GMP:SIL:MexBuildFailed', ...
        'MATLAB mex could not build GMP_SIL_Core:\n%s', ...
        compileError.message);
end

artifact = fullfile(outputDirectory, ['GMP_SIL_Core.' mexext]);
if ~isfile(artifact)
    error('GMP:SIL:MexBuildIncomplete', ...
        'MATLAB mex completed but did not create %s.', artifact);
end
end

function includeRoot = defaultDependencyInclude(repositoryRoot)
switch computer('arch')
    case 'win64'
        triplet = 'x64-windows';
    case 'glnxa64'
        triplet = 'x64-linux';
    otherwise
        error('GMP:SIL:UnsupportedBuildArchitecture', ...
            'No GMP SIL dependency triplet is configured for %s.', ...
            computer('arch'));
end
includeRoot = fullfile(repositoryRoot, 'bin', 'vcpkg_installed', ...
    triplet, triplet, 'include');
end

function validateBuildInputs(sourceFile, dependencyInclude)
if ~isfile(sourceFile)
    error('GMP:SIL:MissingMexSource', ...
        'The canonical GMP SIL MEX source is missing: %s', sourceFile);
end
requiredHeaders = {fullfile(dependencyInclude, 'asio.hpp'), ...
    fullfile(dependencyInclude, 'nlohmann', 'json.hpp')};
if ~all(cellfun(@isfile, requiredHeaders))
    error('GMP:SIL:MissingVcpkgDependencies', ...
        'ASIO or nlohmann-json headers are missing from %s.', ...
        dependencyInclude);
end
end
