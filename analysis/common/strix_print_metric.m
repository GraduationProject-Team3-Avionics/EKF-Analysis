function strix_print_metric(name, x, unit)
    fprintf("%-34s MEAN=%9.4f  RMS=%9.4f  P95=%9.4f  MAX=%9.4f  [%s]\n", ...
        name, strix_meanf(x), strix_rmsf(x), strix_pctf(x, 95), strix_maxf(x), unit);
end
