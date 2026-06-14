function y = strix_meanf(x)
    x = strix_finite(x);
    if isempty(x), y = NaN; else, y = mean(x); end
end
