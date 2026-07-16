function y = ctl_pow(B, Index)
%CTL_POW Preserve float_macros.h semantics: exp(log(Index)*B) = Index^B.
y = single(exp(log(single(Index)) .* single(B)));
end
