function y = strix_minf(x)
    x = strix_finite(x);
    if isempty(x), y = NaN; else, y = min(x); end
end
