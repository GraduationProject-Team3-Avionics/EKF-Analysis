function [raw_acc_norm_g, source] = strix_raw_acc_norm_g(T, cols, N)
    if ismember("raw_acc_norm_g", cols)
        raw_acc_norm_g = double(T.raw_acc_norm_g);
        source = "raw_acc_norm_g";
    elseif all(ismember(["ax", "ay", "az"], cols))
        a = sqrt(double(T.ax).^2 + double(T.ay).^2 + double(T.az).^2);
        % If ax/ay/az look like m/s^2, convert to g. If already g-scale, keep.
        if strix_medianf(abs(a)) > 3.0
            raw_acc_norm_g = a / 9.80665;
            source = "sqrt(ax^2+ay^2+az^2)/g0";
        else
            raw_acc_norm_g = a;
            source = "sqrt(ax^2+ay^2+az^2)";
        end
    else
        raw_acc_norm_g = nan(N, 1);
        source = "not available";
    end
end
