function y = strix_stdf(x)
    x = strix_finite(x);
    if numel(x) < 2, y = NaN; else, y = std(x); end
end
