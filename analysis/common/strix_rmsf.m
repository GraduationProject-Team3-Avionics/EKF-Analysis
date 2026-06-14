function y = strix_rmsf(x)
    x = strix_finite(x);
    if isempty(x), y = NaN; else, y = sqrt(mean(x.^2)); end
end
