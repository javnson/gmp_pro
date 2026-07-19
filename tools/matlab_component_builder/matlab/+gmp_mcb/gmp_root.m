function root = gmp_root()
% Resolve and validate GMP_PRO_LOCATION.
value = getenv('GMP_PRO_LOCATION');
if isempty(value)
    error('GMP:MCB:MissingRoot', 'GMP_PRO_LOCATION is not defined.');
end
root = char(java.io.File(value).getCanonicalPath());
if ~isfile(fullfile(root, 'gmp_core.h')) || ~isfolder(fullfile(root, 'tools', 'gmp_installer'))
    error('GMP:MCB:InvalidRoot', 'GMP_PRO_LOCATION does not identify a GMP repository: %s', root);
end
end

