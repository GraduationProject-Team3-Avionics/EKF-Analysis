function y = strix_maxf(x)
    x = strix_finite(x);
    if isempty(x), y = NaN; else, y = max(x); end
end
