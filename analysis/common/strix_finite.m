function y = strix_finite(x)
%STRIX_FINITE Return finite values only.
    y = x(isfinite(x));
end
