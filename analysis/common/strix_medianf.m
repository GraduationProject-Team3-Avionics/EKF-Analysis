function y = strix_medianf(x)
    x = strix_finite(x);
    if isempty(x), y = NaN; else, y = median(x); end
end
