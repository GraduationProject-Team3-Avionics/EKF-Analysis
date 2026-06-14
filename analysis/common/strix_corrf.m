function y = strix_corrf(a, b)
    idx = isfinite(a) & isfinite(b);
    if sum(idx) < 3
        y = NaN;
    else
        C = corrcoef(a(idx), b(idx));
        y = C(1, 2);
    end
end
