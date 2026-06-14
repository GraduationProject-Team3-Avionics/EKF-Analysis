function [pwm, source] = strix_pwm(T, cols, N)
%STRIX_PWM Load PWM mean signal.
    if ismember("ekf_pwm_mean", cols)
        pwm = double(T.ekf_pwm_mean);
        source = "ekf_pwm_mean";
    elseif all(ismember(["M1", "M2", "M3", "M4"], cols))
        pwm = mean([double(T.M1), double(T.M2), double(T.M3), double(T.M4)], 2, "omitnan");
        source = "mean(M1:M4)";
    else
        pwm = nan(N, 1);
        source = "not available";
    end
end
