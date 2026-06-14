function [gyro_raw, gyro_lpf, gyro_corrected, source] = strix_gyro_norms(T, cols, N)
    deg_to_rad = pi / 180.0;
    gyro_raw = nan(N, 1);
    gyro_lpf = nan(N, 1);
    gyro_corrected = nan(N, 1);
    parts = strings(0);

    if ismember("gyro_raw_norm", cols)
        gyro_raw = double(T.gyro_raw_norm);
        parts(end + 1) = "gyro_raw_norm";
    elseif all(ismember(["gyro_raw_x", "gyro_raw_y", "gyro_raw_z"], cols))
        gyro_raw = sqrt(double(T.gyro_raw_x).^2 + double(T.gyro_raw_y).^2 + double(T.gyro_raw_z).^2);
        parts(end + 1) = "gyro_raw_xyz";
    elseif all(ismember(["gx", "gy", "gz"], cols))
        gnorm = sqrt(double(T.gx).^2 + double(T.gy).^2 + double(T.gz).^2);
        if strix_pctf(abs(gnorm), 95) > 10.0
            gyro_raw = gnorm * deg_to_rad;
            parts(end + 1) = "gx/gy/gz deg/s -> rad/s";
        else
            gyro_raw = gnorm;
            parts(end + 1) = "gx/gy/gz rad/s";
        end
    end

    if ismember("gyro_lpf_norm", cols)
        gyro_lpf = double(T.gyro_lpf_norm);
        parts(end + 1) = "gyro_lpf_norm";
    elseif all(ismember(["gyro_lpf_x", "gyro_lpf_y", "gyro_lpf_z"], cols))
        gyro_lpf = sqrt(double(T.gyro_lpf_x).^2 + double(T.gyro_lpf_y).^2 + double(T.gyro_lpf_z).^2);
        parts(end + 1) = "gyro_lpf_xyz";
    else
        gyro_lpf = gyro_raw;
        parts(end + 1) = "gyro_lpf=fallback_raw";
    end

    if ismember("gyro_corrected_norm", cols)
        gyro_corrected = double(T.gyro_corrected_norm);
        parts(end + 1) = "gyro_corrected_norm";
    elseif all(ismember(["gyro_corrected_x", "gyro_corrected_y", "gyro_corrected_z"], cols))
        gyro_corrected = sqrt(double(T.gyro_corrected_x).^2 + double(T.gyro_corrected_y).^2 + double(T.gyro_corrected_z).^2);
        parts(end + 1) = "gyro_corrected_xyz";
    end

    source = join(parts, " / ");
    if strlength(source) == 0
        source = "not available";
    end
end
