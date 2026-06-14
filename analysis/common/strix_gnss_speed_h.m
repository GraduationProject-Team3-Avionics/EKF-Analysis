function [speed_h, source] = strix_gnss_speed_h(T, cols, N)
    if ismember("gnss_speed_h", cols)
        speed_h = double(T.gnss_speed_h);
        source = "gnss_speed_h";
    elseif all(ismember(["gnss_vel_n_mps", "gnss_vel_e_mps"], cols))
        speed_h = hypot(double(T.gnss_vel_n_mps), double(T.gnss_vel_e_mps));
        source = "hypot(gnss_vel_n_mps, gnss_vel_e_mps)";
    elseif all(ismember(["gnss_vel_n", "gnss_vel_e"], cols))
        speed_h = hypot(double(T.gnss_vel_n), double(T.gnss_vel_e));
        source = "hypot(gnss_vel_n, gnss_vel_e)";
    else
        speed_h = nan(N, 1);
        source = "not available";
    end
end
