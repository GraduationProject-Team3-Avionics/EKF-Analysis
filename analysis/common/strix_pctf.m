function y = strix_pctf(x, p)
    x = strix_finite(x);
    if isempty(x), y = NaN; else, y = prctile(x, p); end
end
