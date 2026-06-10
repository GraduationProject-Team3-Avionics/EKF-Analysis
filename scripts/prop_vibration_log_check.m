clear; clc; close all;

%% STRIX FC prop vibration log check
% Use this after propellers are installed again. The script keeps the
% original Stage 0 velocity checks, but puts the new SD log fields first:
%   raw_acc_norm_g, acc_ned_h, gyro_*_norm, delta_vel_gnss_update_xy,
%   EKF-vs-IMU attitude difference, and accel rotation source.

%% User settings
csv_file = "";
if strlength(string(csv_file)) == 0
    csv_file = string(getenv("STRIX_PROP_CSV"));
end
if strlength(string(csv_file)) == 0
    csv_file = string(getenv("STRIX_STAGE0_CSV"));
end

analysis_start_sec = 0.0;
analysis_end_sec = inf;

use_ekf_ready = true;
use_gnss_ref_ready = true;
use_gnss_valid = false;

pwm_round_step = 50;
min_segment_sec = 2.0;
min_segment_samples = 20;

vel_p95_limit = 0.20;            % m/s
delta_vel_p95_limit = 0.20;      % m/s, only on GNSS update rows
acc_ned_h_warn = 1.0;            % m/s^2
acc_ned_h_fail = 3.0;            % m/s^2
raw_acc_norm_std_warn = 0.10;    % g
raw_acc_norm_std_fail = 0.30;    % g
gyro_lpf_p95_warn = 0.35;        % rad/s
gyro_lpf_p95_fail = 0.60;        % rad/s

top_event_count = 12;
write_summary_csv = true;

%% Select CSV
if strlength(string(csv_file)) == 0
    [file_name, file_path] = uigetfile({"*.CSV;*.csv", "CSV files"}, ...
        "Select STRIX FC prop-on SD log CSV");
    if isequal(file_name, 0)
        error("No CSV selected.");
    end
    csv_file = fullfile(file_path, file_name);
end

csv_file = string(csv_file);
if ~isfile(csv_file)
    error("CSV file not found: %s", csv_file);
end

%% Load table
T = readtable(csv_file, "VariableNamingRule", "preserve");
cols = string(T.Properties.VariableNames);
N = height(T);

need(cols, "timestamp_ms");

t = double(T.timestamp_ms) * 1.0e-3;
t = t - t(1);

mask = isfinite(t) & t >= analysis_start_sec & t <= analysis_end_sec;
if use_ekf_ready && ismember("ekf_ready", cols)
    mask = mask & boolcol(T, cols, "ekf_ready", false(N, 1));
end
if use_gnss_ref_ready && ismember("gnss_ref_ready", cols)
    mask = mask & boolcol(T, cols, "gnss_ref_ready", false(N, 1));
end
if use_gnss_valid && ismember("gnss_valid", cols)
    mask = mask & boolcol(T, cols, "gnss_valid", false(N, 1));
end

%% Signals
[pwm, pwm_source] = load_pwm(T, cols, N);
[vel_xy, vel_source] = load_vel_xy(T, cols, N);
[delta_vel_xy, delta_source] = load_delta_vel_xy(T, cols, N);
[acc_ned_h, acc_ned_source] = load_acc_ned_h(T, cols, N);
[raw_acc_norm_g, raw_acc_source] = load_raw_acc_norm_g(T, cols, N);
[gyro_raw_norm, gyro_lpf_norm, gyro_corrected_norm, gyro_source] = ...
    load_gyro_norms(T, cols, N);

gnss_update = boolcol(T, cols, "gnss_update_executed", false(N, 1));
if ~ismember("gnss_update_executed", cols)
    gnss_update = isfinite(delta_vel_xy) & delta_vel_xy > 1.0e-9;
end
gnss_accepted = boolcol(T, cols, "gnss_correction_accepted", false(N, 1));

roll_diff_deg = col(T, cols, "ekf_imu_roll_diff_deg", nan(N, 1));
pitch_diff_deg = col(T, cols, "ekf_imu_pitch_diff_deg", nan(N, 1));
pitch_diff_rate_deg_s = col(T, cols, "pitch_diff_rate_deg_s", nan(N, 1));
accel_rotation_source = col(T, cols, "accel_rotation_source", nan(N, 1));
accel_rotation_source_used = col(T, cols, "accel_rotation_source_used", nan(N, 1));

motor_on = boolcol(T, cols, "motor_on_detected", false(N, 1));
armed = boolcol(T, cols, "ekf_is_armed", false(N, 1));

%% Overall summary
fprintf("\n=================================================\n");
fprintf("STRIX FC Prop Vibration Log Check\n");
fprintf("=================================================\n");
fprintf("CSV                  : %s\n", csv_file);
fprintf("Rows                 : %d\n", N);
fprintf("Valid mask samples   : %d\n", sum(mask));
fprintf("Time range           : %.3f ~ %.3f sec\n", minf(t(mask)), maxf(t(mask)));
fprintf("PWM source           : %s\n", pwm_source);
fprintf("Velocity source      : %s\n", vel_source);
fprintf("Delta velocity source: %s\n", delta_source);
fprintf("acc_ned_h source     : %s\n", acc_ned_source);
fprintf("raw acc source       : %s\n", raw_acc_source);
fprintf("Gyro source          : %s\n", gyro_source);
fprintf("GNSS update samples  : %d\n", sum(mask & gnss_update));
fprintf("GNSS accepted samples: %d\n", sum(mask & gnss_accepted));
fprintf("Motor-on samples     : %d\n", sum(mask & motor_on));
fprintf("Armed samples        : %d\n", sum(mask & armed));
fprintf("-------------------------------------------------\n");

print_metric("vel_xy", vel_xy(mask), "m/s");
print_metric("delta_vel_xy on GNSS update", delta_vel_xy(mask & gnss_update), "m/s");
print_metric("acc_ned_h", acc_ned_h(mask), "m/s^2");
print_metric("raw_acc_norm_g", raw_acc_norm_g(mask), "g");
print_metric("gyro_raw_norm", gyro_raw_norm(mask), "rad/s");
print_metric("gyro_lpf_norm", gyro_lpf_norm(mask), "rad/s");
print_metric("gyro_corrected_norm", gyro_corrected_norm(mask), "rad/s");
print_metric("pitch_diff_rate", abs(pitch_diff_rate_deg_s(mask)), "deg/s");

overall_vel_p95 = pctf(vel_xy(mask), 95);
overall_delta_p95 = pctf(delta_vel_xy(mask & gnss_update), 95);
overall_acc_p95 = pctf(acc_ned_h(mask), 95);
overall_raw_acc_std = stdf(raw_acc_norm_g(mask));
overall_gyro_lpf_p95 = pctf(gyro_lpf_norm(mask), 95);

fprintf("-------------------------------------------------\n");
fprintf("Velocity status      : %s\n", status_word( ...
    overall_vel_p95 <= vel_p95_limit && overall_delta_p95 <= delta_vel_p95_limit, ...
    overall_vel_p95 <= (vel_p95_limit * 1.5) && overall_delta_p95 <= (delta_vel_p95_limit * 1.5)));
fprintf("Vibration status     : %s\n", vibration_status( ...
    overall_acc_p95, overall_raw_acc_std, overall_gyro_lpf_p95, ...
    acc_ned_h_warn, acc_ned_h_fail, raw_acc_norm_std_warn, raw_acc_norm_std_fail, ...
    gyro_lpf_p95_warn, gyro_lpf_p95_fail));
fprintf("corr(PWM, acc_ned_h) : %.4f\n", corrf(pwm(mask), acc_ned_h(mask)));
fprintf("corr(PWM, vel_xy)    : %.4f\n", corrf(pwm(mask), vel_xy(mask)));
fprintf("corr(acc_ned_h, vel) : %.4f\n", corrf(acc_ned_h(mask), vel_xy(mask)));
fprintf("corr(gyro_lpf, vel)  : %.4f\n", corrf(gyro_lpf_norm(mask), vel_xy(mask)));
fprintf("=================================================\n\n");

%% PWM segments and bins
pwm_round = round(pwm ./ pwm_round_step) .* pwm_round_step;
pwm_round(~isfinite(pwm_round)) = NaN;
segment_base = mask & isfinite(pwm_round);

segments = find_segments(t, segment_base, pwm_round, min_segment_sec, min_segment_samples);
if isempty(segments)
    error("No valid PWM segments found. Reduce min_segment_sec or pwm_round_step.");
end

seg_summary = table();
for k = 1:numel(segments)
    idx = segments(k).idx(:);
    update_idx = idx(gnss_update(idx));

    row = table();
    row.seg_id = k;
    row.pwm_label = segments(k).pwm_label;
    row.pwm_mean = meanf(pwm(idx));
    row.t_start = segments(k).t_start;
    row.t_end = segments(k).t_end;
    row.duration = segments(k).duration;
    row.samples = segments(k).samples;

    row.vel_xy_p95 = pctf(vel_xy(idx), 95);
    row.vel_xy_max = maxf(vel_xy(idx));
    row.delta_update_count = numel(update_idx);
    row.delta_vel_xy_p95 = pctf(delta_vel_xy(update_idx), 95);
    row.delta_vel_xy_max = maxf(delta_vel_xy(update_idx));
    row.gnss_accept_count = sum(gnss_accepted(idx));

    row.acc_ned_h_rms = rmsf(acc_ned_h(idx));
    row.acc_ned_h_p95 = pctf(acc_ned_h(idx), 95);
    row.acc_ned_h_max = maxf(acc_ned_h(idx));
    row.raw_acc_norm_mean_g = meanf(raw_acc_norm_g(idx));
    row.raw_acc_norm_std_g = stdf(raw_acc_norm_g(idx));
    row.raw_acc_norm_p95_g = pctf(raw_acc_norm_g(idx), 95);

    row.gyro_raw_p95 = pctf(gyro_raw_norm(idx), 95);
    row.gyro_lpf_p95 = pctf(gyro_lpf_norm(idx), 95);
    row.gyro_corrected_p95 = pctf(gyro_corrected_norm(idx), 95);
    row.gyro_lpf_to_raw_rms = ratiof(rmsf(gyro_lpf_norm(idx)), rmsf(gyro_raw_norm(idx)));

    row.roll_diff_p95_deg = pctf(abs(roll_diff_deg(idx)), 95);
    row.pitch_diff_p95_deg = pctf(abs(pitch_diff_deg(idx)), 95);
    row.pitch_diff_rate_p95_deg_s = pctf(abs(pitch_diff_rate_deg_s(idx)), 95);
    row.accel_rotation_source_mode = modef(accel_rotation_source(idx));
    row.accel_rotation_source_used_mode = modef(accel_rotation_source_used(idx));

    row.vel_ok = row.vel_xy_p95 <= vel_p95_limit && ...
                 (row.delta_update_count == 0 || row.delta_vel_xy_p95 <= delta_vel_p95_limit);
    row.vibration_warn = row.acc_ned_h_p95 > acc_ned_h_warn || ...
                         row.raw_acc_norm_std_g > raw_acc_norm_std_warn || ...
                         row.gyro_lpf_p95 > gyro_lpf_p95_warn;
    row.vibration_fail = row.acc_ned_h_p95 > acc_ned_h_fail || ...
                         row.raw_acc_norm_std_g > raw_acc_norm_std_fail || ...
                         row.gyro_lpf_p95 > gyro_lpf_p95_fail;
    row.prop_test_pass = row.vel_ok && ~row.vibration_fail;

    seg_summary = [seg_summary; row]; %#ok<AGROW>
end

fprintf("=================================================\n");
fprintf("PWM Segment Summary\n");
fprintf("=================================================\n");
disp(seg_summary);

labels = unique(pwm_round(segment_base));
bin_summary = table();
for n = 1:numel(labels)
    idx = find(segment_base & pwm_round == labels(n));
    update_idx = idx(gnss_update(idx));

    row = table();
    row.pwm_label = labels(n);
    row.samples = numel(idx);
    row.duration_approx = maxf(t(idx)) - minf(t(idx));
    row.pwm_mean = meanf(pwm(idx));
    row.vel_xy_p95 = pctf(vel_xy(idx), 95);
    row.delta_update_count = numel(update_idx);
    row.delta_vel_xy_p95 = pctf(delta_vel_xy(update_idx), 95);
    row.acc_ned_h_rms = rmsf(acc_ned_h(idx));
    row.acc_ned_h_p95 = pctf(acc_ned_h(idx), 95);
    row.acc_ned_h_max = maxf(acc_ned_h(idx));
    row.raw_acc_norm_std_g = stdf(raw_acc_norm_g(idx));
    row.gyro_lpf_p95 = pctf(gyro_lpf_norm(idx), 95);
    row.pitch_diff_rate_p95_deg_s = pctf(abs(pitch_diff_rate_deg_s(idx)), 95);
    row.vibration_fail = row.acc_ned_h_p95 > acc_ned_h_fail || ...
                         row.raw_acc_norm_std_g > raw_acc_norm_std_fail || ...
                         row.gyro_lpf_p95 > gyro_lpf_p95_fail;

    bin_summary = [bin_summary; row]; %#ok<AGROW>
end

fprintf("=================================================\n");
fprintf("PWM Bin Summary\n");
fprintf("=================================================\n");
disp(bin_summary);

bad_bins = bin_summary.pwm_label(bin_summary.vibration_fail);
if isempty(bad_bins)
    fprintf("No PWM bin exceeded the hard vibration limits.\n\n");
else
    fprintf("Hard vibration bins: %s us\n\n", join(string(bad_bins'), ", "));
end

%% Worst event prints
fprintf("=================================================\n");
fprintf("Worst Events\n");
fprintf("=================================================\n");
print_top_events("acc_ned_h", acc_ned_h, mask, top_event_count, ...
    t, pwm, vel_xy, delta_vel_xy, acc_ned_h, raw_acc_norm_g, gyro_lpf_norm, ...
    pitch_diff_rate_deg_s, gnss_update, gnss_accepted);
print_top_events("gyro_lpf_norm", gyro_lpf_norm, mask, top_event_count, ...
    t, pwm, vel_xy, delta_vel_xy, acc_ned_h, raw_acc_norm_g, gyro_lpf_norm, ...
    pitch_diff_rate_deg_s, gnss_update, gnss_accepted);
print_top_events("delta_vel_xy", delta_vel_xy, mask & gnss_update, top_event_count, ...
    t, pwm, vel_xy, delta_vel_xy, acc_ned_h, raw_acc_norm_g, gyro_lpf_norm, ...
    pitch_diff_rate_deg_s, gnss_update, gnss_accepted);
fprintf("=================================================\n\n");

%% Save summaries
if write_summary_csv
    [csv_dir, base_name, ~] = fileparts(csv_file);

    % CSV 파일이 있는 폴더 바깥/또는 현재 작업 폴더에 results 폴더 생성
    results_dir = fullfile(pwd, "results");

    if ~exist(results_dir, "dir")
        mkdir(results_dir);
    end

    seg_path = fullfile(results_dir, base_name + "_prop_vibration_segments.csv");
    bin_path = fullfile(results_dir, base_name + "_prop_vibration_pwm_bins.csv");

    writetable(seg_summary, seg_path);
    writetable(bin_summary, bin_path);

    fprintf("Saved segment summary: %s\n", seg_path);
    fprintf("Saved PWM bin summary: %s\n", bin_path);
end

%% Plots
figure("Name", "Prop vibration time series", "Color", "w");
tiledlayout(5, 1, "TileSpacing", "compact");

nexttile;
plot(t, pwm, "LineWidth", 1.0); grid on;
ylabel("PWM [us]");
title("PWM");

nexttile;
plot(t, vel_xy, "LineWidth", 1.0); hold on;
yline(vel_p95_limit, "--r", "vel P95 limit");
grid on; ylabel("vel XY [m/s]");
title("EKF horizontal velocity");

nexttile;
plot(t, acc_ned_h, "LineWidth", 1.0); hold on;
yline(acc_ned_h_warn, "--", "warn");
yline(acc_ned_h_fail, "--r", "fail");
grid on; ylabel("acc NED H [m/s^2]");
title("Horizontal acceleration after attitude rotation");

nexttile;
plot(t, raw_acc_norm_g, "LineWidth", 1.0); grid on;
ylabel("|acc raw| [g]");
title("Raw accelerometer norm");

nexttile;
plot(t, gyro_raw_norm, "Color", [0.65 0.65 0.65], "LineWidth", 0.8); hold on;
plot(t, gyro_lpf_norm, "LineWidth", 1.0);
plot(t, gyro_corrected_norm, "LineWidth", 1.0);
yline(gyro_lpf_p95_warn, "--", "warn");
yline(gyro_lpf_p95_fail, "--r", "fail");
grid on; xlabel("time [s]"); ylabel("gyro [rad/s]");
legend("raw", "lpf", "corrected", "Location", "best");
title("Gyro norm");

figure("Name", "PWM vibration bins", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");
x = categorical(string(bin_summary.pwm_label));
x = reordercats(x, string(bin_summary.pwm_label));

nexttile;
bar(x, bin_summary.acc_ned_h_p95); hold on;
yline(acc_ned_h_warn, "--");
yline(acc_ned_h_fail, "--r");
grid on; ylabel("m/s^2"); title("acc_ned_h P95");

nexttile;
bar(x, bin_summary.raw_acc_norm_std_g); hold on;
yline(raw_acc_norm_std_warn, "--");
yline(raw_acc_norm_std_fail, "--r");
grid on; ylabel("g"); title("raw_acc_norm_g std");

nexttile;
bar(x, bin_summary.gyro_lpf_p95); hold on;
yline(gyro_lpf_p95_warn, "--");
yline(gyro_lpf_p95_fail, "--r");
grid on; ylabel("rad/s"); title("gyro_lpf_norm P95");

nexttile;
bar(x, bin_summary.vel_xy_p95); hold on;
yline(vel_p95_limit, "--r");
grid on; ylabel("m/s"); title("vel_xy P95");

figure("Name", "Vibration coupling", "Color", "w");
tiledlayout(1, 3, "TileSpacing", "compact");
nexttile;
scatter(acc_ned_h(mask), vel_xy(mask), 12, pwm(mask), "filled");
grid on; xlabel("acc_ned_h [m/s^2]"); ylabel("vel_xy [m/s]");
title("acc_ned_h vs velocity"); colorbar;
nexttile;
scatter(gyro_lpf_norm(mask), vel_xy(mask), 12, pwm(mask), "filled");
grid on; xlabel("gyro_lpf_norm [rad/s]"); ylabel("vel_xy [m/s]");
title("gyro_lpf vs velocity"); colorbar;
nexttile;
scatter(raw_acc_norm_g(mask), acc_ned_h(mask), 12, pwm(mask), "filled");
grid on; xlabel("raw_acc_norm_g [g]"); ylabel("acc_ned_h [m/s^2]");
title("raw acc norm vs rotated acc"); colorbar;

%% Local functions
function need(cols, names)
    missing = string.empty;
    for i = 1:numel(names)
        if ~ismember(names(i), cols)
            missing(end + 1) = names(i); %#ok<AGROW>
        end
    end
    if ~isempty(missing)
        error("Missing required column(s): %s", join(missing, ", "));
    end
end

function y = col(T, cols, name, default_value)
    if ismember(name, cols)
        y = double(T.(name));
    else
        y = default_value;
    end
end

function y = boolcol(T, cols, name, default_value)
    if ismember(name, cols)
        y = double(T.(name)) ~= 0;
    else
        y = default_value;
    end
end

function [pwm, source] = load_pwm(T, cols, N)
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

function [vel_xy, source] = load_vel_xy(T, cols, N)
    if ismember("ekf_speed_h", cols)
        vel_xy = double(T.ekf_speed_h);
        source = "ekf_speed_h";
    elseif all(ismember(["ekf_vel_n", "ekf_vel_e"], cols))
        vel_xy = hypot(double(T.ekf_vel_n), double(T.ekf_vel_e));
        source = "hypot(ekf_vel_n, ekf_vel_e)";
    else
        vel_xy = nan(N, 1);
        source = "not available";
    end
end

function [delta_vel_xy, source] = load_delta_vel_xy(T, cols, N)
    if ismember("delta_vel_gnss_update_xy", cols)
        delta_vel_xy = double(T.delta_vel_gnss_update_xy);
        source = "delta_vel_gnss_update_xy";
    elseif all(ismember(["delta_vel_gnss_update_n", "delta_vel_gnss_update_e"], cols))
        delta_vel_xy = hypot(double(T.delta_vel_gnss_update_n), double(T.delta_vel_gnss_update_e));
        source = "hypot(delta_vel_gnss_update_n/e)";
    else
        delta_vel_xy = nan(N, 1);
        source = "not available";
    end
end

function [acc_ned_h, source] = load_acc_ned_h(T, cols, N)
    if ismember("acc_ned_h", cols)
        acc_ned_h = double(T.acc_ned_h);
        source = "acc_ned_h";
    elseif all(ismember(["acc_ned_n", "acc_ned_e"], cols))
        acc_ned_h = hypot(double(T.acc_ned_n), double(T.acc_ned_e));
        source = "hypot(acc_ned_n, acc_ned_e)";
    else
        acc_ned_h = nan(N, 1);
        source = "not available";
    end
end

function [raw_acc_norm_g, source] = load_raw_acc_norm_g(T, cols, N)
    if ismember("raw_acc_norm_g", cols)
        raw_acc_norm_g = double(T.raw_acc_norm_g);
        source = "raw_acc_norm_g";
    elseif all(ismember(["ax", "ay", "az"], cols))
        raw_acc_norm_g = sqrt(double(T.ax).^2 + double(T.ay).^2 + double(T.az).^2);
        source = "sqrt(ax^2+ay^2+az^2)";
    else
        raw_acc_norm_g = nan(N, 1);
        source = "not available";
    end
end

function [gyro_raw, gyro_lpf, gyro_corrected, source] = load_gyro_norms(T, cols, N)
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
        gyro_raw = sqrt(double(T.gx).^2 + double(T.gy).^2 + double(T.gz).^2) * deg_to_rad;
        parts(end + 1) = "gx/gy/gz";
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

function segments = find_segments(t, segment_base, pwm_round, min_segment_sec, min_segment_samples)
    N = numel(t);
    segments = struct([]);
    seg_count = 0;
    i = 1;
    while i <= N
        if ~segment_base(i)
            i = i + 1;
            continue;
        end
        current_pwm = pwm_round(i);
        j = i;
        while j <= N && segment_base(j) && pwm_round(j) == current_pwm
            j = j + 1;
        end
        idx = i:(j - 1);
        duration = t(idx(end)) - t(idx(1));
        if duration >= min_segment_sec && numel(idx) >= min_segment_samples
            seg_count = seg_count + 1;
            segments(seg_count).idx = idx; %#ok<AGROW>
            segments(seg_count).pwm_label = current_pwm;
            segments(seg_count).t_start = t(idx(1));
            segments(seg_count).t_end = t(idx(end));
            segments(seg_count).duration = duration;
            segments(seg_count).samples = numel(idx);
        end
        i = j;
    end
end

function y = finite_vec(x)
    y = x(isfinite(x));
end

function y = meanf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = mean(x); end
end

function y = minf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = min(x); end
end

function y = maxf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = max(x); end
end

function y = rmsf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = sqrt(mean(x.^2)); end
end

function y = stdf(x)
    x = finite_vec(x);
    if numel(x) < 2, y = NaN; else, y = std(x); end
end

function y = pctf(x, p)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = prctile(x, p); end
end

function y = corrf(a, b)
    idx = isfinite(a) & isfinite(b);
    if sum(idx) < 3
        y = NaN;
    else
        C = corrcoef(a(idx), b(idx));
        y = C(1, 2);
    end
end

function y = ratiof(num, den)
    if isfinite(num) && isfinite(den) && abs(den) > 1.0e-9
        y = num / den;
    else
        y = NaN;
    end
end

function y = modef(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = mode(x); end
end

function print_metric(name, x, unit)
    fprintf("%-30s RMS=%8.4f  P95=%8.4f  MAX=%8.4f  MEAN=%8.4f  [%s]\n", ...
        name, rmsf(x), pctf(x, 95), maxf(x), meanf(x), unit);
end

function s = status_word(pass_ok, warn_ok)
    if pass_ok
        s = "PASS";
    elseif warn_ok
        s = "WARN";
    else
        s = "FAIL";
    end
end

function s = vibration_status(acc_p95, raw_acc_std, gyro_lpf_p95, ...
                              acc_warn, acc_fail, raw_warn, raw_fail, ...
                              gyro_warn, gyro_fail)
    fail = acc_p95 > acc_fail || raw_acc_std > raw_fail || gyro_lpf_p95 > gyro_fail;
    warn = acc_p95 > acc_warn || raw_acc_std > raw_warn || gyro_lpf_p95 > gyro_warn;
    if fail
        s = "FAIL";
    elseif warn
        s = "WARN";
    else
        s = "PASS";
    end
end

function print_top_events(name, score, event_mask, top_n, ...
                          t, pwm, vel_xy, delta_vel_xy, acc_ned_h, ...
                          raw_acc_norm_g, gyro_lpf_norm, pitch_diff_rate, ...
                          gnss_update, gnss_accepted)
    idx = find(event_mask & isfinite(score));
    if isempty(idx)
        fprintf("%s top events: none\n\n", name);
        return;
    end

    [~, order] = sort(score(idx), "descend");
    idx = idx(order(1:min(top_n, numel(order))));

    fprintf("%s top events:\n", name);
    fprintf("  rank   t[s]      PWM     vel_xy   dV_gnss  acc_ned_h  raw_acc_g  gyro_lpf  pitch_rate  upd acc\n");
    for k = 1:numel(idx)
        ii = idx(k);
        fprintf("  %4d %8.3f %8.1f %8.4f %8.4f %10.4f %10.4f %9.4f %11.4f  %3d %3d\n", ...
            k, t(ii), pwm(ii), vel_xy(ii), delta_vel_xy(ii), acc_ned_h(ii), ...
            raw_acc_norm_g(ii), gyro_lpf_norm(ii), abs(pitch_diff_rate(ii)), ...
            gnss_update(ii), gnss_accepted(ii));
    end
    fprintf("\n");
end
