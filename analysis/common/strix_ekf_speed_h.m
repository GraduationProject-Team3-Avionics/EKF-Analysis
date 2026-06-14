function [speed_h, source] = strix_ekf_speed_h(T, cols, N)
    if ismember("ekf_speed_h", cols)
        speed_h = double(T.ekf_speed_h);
        source = "ekf_speed_h";
    elseif all(ismember(["ekf_vel_n", "ekf_vel_e"], cols))
        speed_h = hypot(double(T.ekf_vel_n), double(T.ekf_vel_e));
        source = "hypot(ekf_vel_n, ekf_vel_e)";
    else
        speed_h = nan(N, 1);
        source = "not available";
    end
end
