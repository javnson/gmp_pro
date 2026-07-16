function y = abs_static_inline(A)
%ABS_STATIC_INLINE MATLAB equivalent of the float backend helper.
A = single(A);
y = A;
negative = A < 0;
y(negative) = -A(negative);
end
