clear; clc; close all;

%% STRIX FC Stage 0 velocity log check
% Use this for tied-down / fixed-frame logs before any XY velocity or
% position hold test. It focuses on the current question:
%   PWM -> fake EKF velocity?
%   GNSS update -> large velocity state jump?
%   acc_ned / gyro -> vibration or attitude-propagation disturbance?

%% User settings
csv_file = "";
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

vel_rms_limit = 0.10;       % m/s, Stage 0 example limit
vel_p95_limit = 0.20;       % m/s, Stage 0 example limit
delta_rms_limit = 0.10;     % m/s, Stage 0 example limit
delta_p95_limit = 0.20;     % m/s, Stage 0 example limit

write_summary_csv = true;

%% Select CSV
if strlength(string(csv_file)) == 0
    [file_name, file_path] = uigetfile({"*.CSV;*.csv", "CSV files"}, ...
        "Select STRIX FC SD log CSV");
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
need(cols, ["ekf_vel_n", "ekf_vel_e"]);

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
pwm = load_pwm(T, cols, N);
pwm_source = "ekf_pwm_mean";
if ~ismember("ekf_pwm_mean", cols)
    pwm_source = "mean(M1:M4)";
end

vel_n = double(T.ekf_vel_n);
vel_e = double(T.ekf_vel_e);
vel_xy = hypot(vel_n, vel_e);

delta_vel_n = col(T, cols, "delta_vel_gnss_update_n", nan(N, 1));
delta_vel_e = col(T, cols, "delta_vel_gnss_update_e", nan(N, 1));
if ismember("delta_vel_gnss_update_xy", cols)
    delta_vel_xy = col(T, cols, "delta_vel_gnss_update_xy", nan(N, 1));
else
    delta_vel_xy = hypot(delta_vel_n, delta_vel_e);
end

gnss_update = boolcol(T, cols, "gnss_update_executed", false(N, 1));
if ~ismember("gnss_update_executed", cols)
    gnss_update = isfinite(delta_vel_xy) & delta_vel_xy > 1.0e-9;
end
gnss_accepted = boolcol(T, cols, "gnss_correction_accepted", false(N, 1));
gnss_age_ms = col(T, cols, "gnss_last_update_age_ms", nan(N, 1));
gnss_update_dt = col(T, cols, "gnss_update_dt", nan(N, 1));

acc_ned_n = col(T, cols, "acc_ned_n", nan(N, 1));
acc_ned_e = col(T, cols, "acc_ned_e", nan(N, 1));
if ismember("acc_ned_h", cols)
    acc_ned_h = col(T, cols, "acc_ned_h", nan(N, 1));
else
    acc_ned_h = hypot(acc_ned_n, acc_ned_e);
end

raw_acc_norm_g = nan(N, 1);
if ismember("raw_acc_norm_g", cols)
    raw_acc_norm_g = col(T, cols, "raw_acc_norm_g", nan(N, 1));
elseif all(ismember(["ax", "ay", "az"], cols))
    raw_acc_norm_g = sqrt(double(T.ax).^2 + double(T.ay).^2 + double(T.az).^2);
end

[gyro_norm_rad_s, gyro_source] = load_gyro_norm_rad_s(T, cols, N);

innov_vel_n = col(T, cols, "ekf_innov_vel_n", nan(N, 1));
innov_vel_e = col(T, cols, "ekf_innov_vel_e", nan(N, 1));
innov_vel_xy = hypot(innov_vel_n, innov_vel_e);

R_vel_n = col(T, cols, "ekf_R_applied_gnss_vel_n", nan(N, 1));
R_vel_e = col(T, cols, "ekf_R_applied_gnss_vel_e", nan(N, 1));
sigma_vel_xy = sqrt(max(0.5 .* (R_vel_n + R_vel_e), 0));

%% Overall summary
fprintf("\n=================================================\n");
fprintf("STRIX FC Stage 0 Velocity Log Check\n");
fprintf("=================================================\n");
fprintf("CSV                  : %s\n", csv_file);
fprintf("Rows                 : %d\n", N);
fprintf("Valid mask samples   : %d\n", sum(mask));
fprintf("Time range           : %.3f ~ %.3f sec\n", minf(t(mask)), maxf(t(mask)));
fprintf("PWM source           : %s\n", pwm_source);
fprintf("Gyro source          : %s\n", gyro_source);
fprintf("GNSS update samples  : %d\n", sum(mask & gnss_update));
fprintf("GNSS accepted samples: %d\n", sum(mask & gnss_accepted));
fprintf("-------------------------------------------------\n");

print_metric("vel_xy all", vel_xy(mask), "m/s");
print_metric("delta_vel_xy on GNSS update", delta_vel_xy(mask & gnss_update), "m/s");
print_metric("acc_ned_h", acc_ned_h(mask), "m/s^2");
print_metric("gyro_norm", gyro_norm_rad_s(mask), "rad/s");
print_metric("raw_acc_norm_g", raw_acc_norm_g(mask), "g");

fprintf("-------------------------------------------------\n");
fprintf("corr(PWM, vel_xy)       : %.4f\n", corrf(pwm(mask), vel_xy(mask)));
fprintf("corr(PWM, acc_ned_h)    : %.4f\n", corrf(pwm(mask), acc_ned_h(mask)));
fprintf("corr(acc_ned_h, vel_xy) : %.4f\n", corrf(acc_ned_h(mask), vel_xy(mask)));
fprintf("corr(gyro_norm, vel_xy) : %.4f\n", corrf(gyro_norm_rad_s(mask), vel_xy(mask)));
fprintf("=================================================\n\n");

%% PWM segment detection
pwm_round = round(pwm ./ pwm_round_step) .* pwm_round_step;
pwm_round(~isfinite(pwm_round)) = NaN;

segment_base = mask & isfinite(pwm_round);
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
        segments(seg_count).idx = idx; %#ok<SAGROW>
        segments(seg_count).pwm_label = current_pwm;
        segments(seg_count).t_start = t(idx(1));
        segments(seg_count).t_end = t(idx(end));
        segments(seg_count).duration = duration;
        segments(seg_count).samples = numel(idx);
    end

    i = j;
end

if isempty(segments)
    error("No valid PWM segments found. Reduce min_segment_sec or pwm_round_step.");
end

%% Segment table
seg_summary = table();

for k = 1:numel(segments)
    idx = segments(k).idx(:);
    update_idx = idx(gnss_update(idx));

    row = table();
    row.seg_id = k;
    row.pwm_label = segments(k).pwm_label;
    row.pwm_mean = meanf(pwm(idx));
    row.pwm_min = minf(pwm(idx));
    row.pwm_max = maxf(pwm(idx));
    row.t_start = segments(k).t_start;
    row.t_end = segments(k).t_end;
    row.duration = segments(k).duration;
    row.samples = segments(k).samples;

    row.vel_xy_rms = rmsf(vel_xy(idx));
    row.vel_xy_p95 = pctf(vel_xy(idx), 95);
    row.vel_xy_max = maxf(vel_xy(idx));
    row.vel_xy_mean = meanf(vel_xy(idx));

    row.delta_update_count = numel(update_idx);
    row.delta_vel_xy_rms = rmsf(delta_vel_xy(update_idx));
    row.delta_vel_xy_p95 = pctf(delta_vel_xy(update_idx), 95);
    row.delta_vel_xy_max = maxf(delta_vel_xy(update_idx));
    row.gnss_accept_count = sum(gnss_accepted(idx));

    row.acc_ned_h_rms = rmsf(acc_ned_h(idx));
    row.acc_ned_h_p95 = pctf(acc_ned_h(idx), 95);
    row.acc_ned_h_max = maxf(acc_ned_h(idx));

    row.gyro_norm_rms = rmsf(gyro_norm_rad_s(idx));
    row.gyro_norm_p95 = pctf(gyro_norm_rad_s(idx), 95);
    row.gyro_norm_max = maxf(gyro_norm_rad_s(idx));

    row.raw_acc_norm_std_g = stdf(raw_acc_norm_g(idx));
    row.innov_vel_xy_p95 = pctf(innov_vel_xy(update_idx), 95);
    row.sigma_vel_xy_mean = meanf(sigma_vel_xy(update_idx));
    row.gnss_age_ms_mean = meanf(gnss_age_ms(update_idx));
    row.gnss_update_dt_mean = meanf(gnss_update_dt(update_idx));

    row.vel_ok = row.vel_xy_rms < vel_rms_limit && row.vel_xy_p95 < vel_p95_limit;
    row.delta_ok = row.delta_update_count > 0 && ...
                   row.delta_vel_xy_rms < delta_rms_limit && ...
                   row.delta_vel_xy_p95 < delta_p95_limit;
    row.stage0_pass = row.vel_ok && row.delta_ok;

    seg_summary = [seg_summary; row]; %#ok<AGROW>
end

fprintf("=================================================\n");
fprintf("PWM Segment Summary\n");
fprintf("=================================================\n");
disp(seg_summary);

%% Aggregated PWM bin table
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
    row.vel_xy_rms = rmsf(vel_xy(idx));
    row.vel_xy_p95 = pctf(vel_xy(idx), 95);
    row.vel_xy_max = maxf(vel_xy(idx));
    row.delta_update_count = numel(update_idx);
    row.delta_vel_xy_rms = rmsf(delta_vel_xy(update_idx));
    row.delta_vel_xy_p95 = pctf(delta_vel_xy(update_idx), 95);
    row.delta_vel_xy_max = maxf(delta_vel_xy(update_idx));
    row.acc_ned_h_rms = rmsf(acc_ned_h(idx));
    row.gyro_norm_rms = rmsf(gyro_norm_rad_s(idx));
    row.raw_acc_norm_std_g = stdf(raw_acc_norm_g(idx));

    bin_summary = [bin_summary; row]; %#ok<AGROW>
end

fprintf("=================================================\n");
fprintf("PWM Bin Summary\n");
fprintf("=================================================\n");
disp(bin_summary);

%% Worst events
fprintf("=================================================\n");
fprintf("Worst Velocity Events\n");
fprintf("=================================================\n");
print_top_events("vel_xy", vel_xy, mask, t, pwm, vel_xy, delta_vel_xy, acc_ned_h, gyro_norm_rad_s, gnss_update, gnss_accepted);
fprintf("\n");
print_top_events("delta_vel_xy", delta_vel_xy, mask & gnss_update, t, pwm, vel_xy, delta_vel_xy, acc_ned_h, gyro_norm_rad_s, gnss_update, gnss_accepted);
fprintf("=================================================\n\n");

%% Save summaries
if write_summary_csv
    [csv_dir, base_name, ~] = fileparts(csv_file);

    % 입력 CSV가 있는 폴더 안에 results 폴더 생성
    results_dir = fullfile(csv_dir, "results");

    if ~exist(results_dir, "dir")
        mkdir(results_dir);
    end

    seg_path = fullfile(results_dir, base_name + "_stage0_segments.csv");
    bin_path = fullfile(results_dir, base_name + "_stage0_pwm_bins.csv");

    writetable(seg_summary, seg_path);
    writetable(bin_summary, bin_path);

    fprintf("Saved segment summary: %s\n", seg_path);
    fprintf("Saved PWM bin summary: %s\n", bin_path);
end

%% Figures
figure("Name", "Stage0 overview", "Color", "w");
tiledlayout(5, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, pwm, "LineWidth", 1.0); grid on;
ylabel("PWM [us]");
title("PWM");

nexttile;
plot(t, vel_xy, "LineWidth", 1.0); hold on; grid on;
yline(vel_rms_limit, "--", "0.10");
yline(vel_p95_limit, ":", "0.20");
ylabel("m/s");
title("EKF horizontal velocity magnitude");

nexttile;
plot(t, delta_vel_xy, "LineWidth", 1.0); hold on; grid on;
scatter(t(gnss_update), delta_vel_xy(gnss_update), 16, "filled");
yline(delta_rms_limit, "--", "0.10");
yline(delta_p95_limit, ":", "0.20");
ylabel("m/s");
title("GNSS update velocity jump");

nexttile;
plot(t, acc_ned_h, "LineWidth", 1.0); grid on;
ylabel("m/s^2");
title("acc_ned horizontal");

nexttile;
plot(t, gyro_norm_rad_s, "LineWidth", 1.0); grid on;
ylabel("rad/s");
xlabel("Time [s]");
title("gyro norm");

figure("Name", "Stage0 PWM segment metrics", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact", "Padding", "compact");

x_labels = strings(height(seg_summary), 1);
for i = 1:height(seg_summary)
    x_labels(i) = sprintf("S%d %.0f", seg_summary.seg_id(i), seg_summary.pwm_mean(i));
end
x = categorical(x_labels);
x = reordercats(x, x_labels);

nexttile;
bar(x, [seg_summary.vel_xy_rms, seg_summary.vel_xy_p95]);
grid on; ylabel("m/s");
title("vel_xy by PWM segment");
legend("RMS", "95%", "Location", "best");

nexttile;
bar(x, [seg_summary.delta_vel_xy_rms, seg_summary.delta_vel_xy_p95]);
grid on; ylabel("m/s");
title("delta_vel_gnss_update_xy by PWM segment");
legend("RMS", "95%", "Location", "best");

nexttile;
bar(x, seg_summary.acc_ned_h_rms);
grid on; ylabel("m/s^2");
title("acc_ned_h RMS by PWM segment");

nexttile;
bar(x, seg_summary.gyro_norm_rms);
grid on; ylabel("rad/s");
xlabel("PWM segment");
title("gyro norm RMS by PWM segment");

figure("Name", "Stage0 scatter checks", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
scatter(pwm(mask), vel_xy(mask), 12, "filled"); grid on;
xlabel("PWM [us]"); ylabel("vel_xy [m/s]");
title("PWM vs EKF velocity");

nexttile;
scatter(acc_ned_h(mask), vel_xy(mask), 12, "filled"); grid on;
xlabel("acc_ned_h [m/s^2]"); ylabel("vel_xy [m/s]");
title("acc_ned_h vs EKF velocity");

nexttile;
scatter(gyro_norm_rad_s(mask), vel_xy(mask), 12, "filled"); grid on;
xlabel("gyro norm [rad/s]"); ylabel("vel_xy [m/s]");
title("gyro vs EKF velocity");

nexttile;
update_mask = mask & gnss_update;
scatter(innov_vel_xy(update_mask), delta_vel_xy(update_mask), 16, "filled"); grid on;
xlabel("innov_vel_xy [m/s]"); ylabel("delta_vel_xy [m/s]");
title("GNSS innovation vs velocity jump");

%% Local functions
function need(cols, names)
    names = string(names);
    for ii = 1:numel(names)
        if ~ismember(names(ii), cols)
            error("Required column missing: %s", names(ii));
        end
    end
end

function x = col(T, cols, name, default_value)
    name = string(name);
    if ismember(name, cols)
        x = double(T.(name));
    else
        x = default_value;
    end
end

function x = boolcol(T, cols, name, default_value)
    name = string(name);
    if ismember(name, cols)
        x = double(T.(name)) ~= 0;
    else
        x = default_value;
    end
end

function pwm = load_pwm(T, cols, N)
    if ismember("ekf_pwm_mean", cols)
        pwm = double(T.ekf_pwm_mean);
    elseif all(ismember(["M1", "M2", "M3", "M4"], cols))
        pwm = mean([double(T.M1), double(T.M2), double(T.M3), double(T.M4)], 2, "omitnan");
    else
        error("PWM column missing: need ekf_pwm_mean or M1/M2/M3/M4.");
    end
    if numel(pwm) ~= N
        error("Internal PWM length mismatch.");
    end
end

function [gyro_norm, source] = load_gyro_norm_rad_s(T, cols, N)
    deg_to_rad = pi / 180.0;
    if ismember("gyro_lpf_norm", cols)
        gyro_norm = double(T.gyro_lpf_norm);
        source = "gyro_lpf_norm rad/s";
        return;
    elseif ismember("gyro_raw_norm", cols)
        gyro_norm = double(T.gyro_raw_norm);
        source = "gyro_raw_norm rad/s";
        return;
    elseif all(ismember(["gyro_lpf_x", "gyro_lpf_y", "gyro_lpf_z"], cols))
        gx = double(T.gyro_lpf_x);
        gy = double(T.gyro_lpf_y);
        gz = double(T.gyro_lpf_z);
        source = "gyro_lpf_x/y/z rad/s";
    elseif all(ismember(["gyro_raw_x", "gyro_raw_y", "gyro_raw_z"], cols))
        gx = double(T.gyro_raw_x);
        gy = double(T.gyro_raw_y);
        gz = double(T.gyro_raw_z);
        source = "gyro_raw_x/y/z rad/s";
    elseif all(ismember(["gx", "gy", "gz"], cols))
        gx = double(T.gx) * deg_to_rad;
        gy = double(T.gy) * deg_to_rad;
        gz = double(T.gz) * deg_to_rad;
        source = "gx/gy/gz deg/s converted to rad/s";
    else
        gyro_norm = nan(N, 1);
        source = "not available";
        return;
    end
    gyro_norm = sqrt(gx.^2 + gy.^2 + gz.^2);
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
    x = sort(finite_vec(x));
    if isempty(x)
        y = NaN;
        return;
    end
    if numel(x) == 1
        y = x(1);
        return;
    end
    q = 1 + (numel(x) - 1) * p / 100.0;
    lo = floor(q);
    hi = ceil(q);
    if lo == hi
        y = x(lo);
    else
        y = x(lo) + (x(hi) - x(lo)) * (q - lo);
    end
end

function r = corrf(a, b)
    valid = isfinite(a) & isfinite(b);
    a = a(valid);
    b = b(valid);
    if numel(a) < 3 || std(a) <= 0 || std(b) <= 0
        r = NaN;
        return;
    end
    c = corrcoef(a, b);
    r = c(1, 2);
end

function print_metric(name, x, unit)
    fprintf("%-30s RMS=%8.4f  P95=%8.4f  MAX=%8.4f  MEAN=%8.4f  [%s]\n", ...
        name, rmsf(x), pctf(x, 95), maxf(x), meanf(x), unit);
end

function print_top_events(name, score, event_mask, t, pwm, vel_xy, delta_vel_xy, acc_ned_h, gyro_norm, gnss_update, gnss_accepted)
    idx = find(event_mask & isfinite(score));
    if isempty(idx)
        fprintf("%s: no valid events\n", name);
        return;
    end
    [~, order] = sort(score(idx), "descend");
    idx = idx(order(1:min(10, numel(order))));

    fprintf("%s top events:\n", name);
    fprintf("  rank   t[s]      PWM     vel_xy   dV_gnss  acc_ned_h  gyro_norm  upd acc\n");
    for k = 1:numel(idx)
        ii = idx(k);
        fprintf("  %4d %8.3f %8.1f %8.4f %8.4f %10.4f %10.4f  %3d %3d\n", ...
            k, t(ii), pwm(ii), vel_xy(ii), delta_vel_xy(ii), acc_ned_h(ii), ...
            gyro_norm(ii), gnss_update(ii), gnss_accepted(ii));
    end
end
