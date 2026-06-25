clear; clc; close all;

%% STRIX Fixed Ground PWM1300 Hold Analysis
% Expected test:
%   1000 PWM baseline -> 1300 PWM hold -> 1000 PWM recovery
%
% Purpose:
%   - 1300 PWM에서 IMU vertical acceleration noise가 지속적으로 큰지 확인
%   - 1300 PWM에서 barometer residual이 증가하는지 확인
%   - 1300 hold 이후 1000 recovery에서 baro/IMU가 회복되는지 확인
%   - Q_acc_z, R_baro PWM scale 근거 확인
%
% Note:
%   ekf_innov_pos_d는 baro innovation이 아니라 GNSS D position innovation으로 해석해야 함.

%% User settings
csv_file = "data/4OUTDOOR_FIXED_PWM1300_HOLD_GROUND.CSV";

if ~isfile(csv_file)
    csv_file = "4OUTDOOR_FIXED_PWM1300_HOLD_GROUND.CSV";
end

pwm_round_step = 50;
baseline_pwm = 1000;
target_pwm = 1300;

% PWM 전환 직후/직전 과도응답 제거
ignore_segment_edge_sec = 3.0;

% 1300 hold 구간 안에서 더 안정적인 중앙부만 볼 때 사용
use_middle_ratio_for_hold = true;
hold_middle_ratio = 0.70;   % 중앙 70%만 steady hold로 사용

% residual window
baro_residual_window_sec = 5.0;
acc_residual_window_sec = 2.0;

% short segment 판단
min_segment_sec = 8.0;
min_used_samples = 30;

%% Load
T = readtable(csv_file);

fprintf("\n========================================\n");
fprintf("STRIX PWM1300 Hold Analysis\n");
fprintf("File: %s\n", csv_file);
fprintf("========================================\n");

%% Time
t = double(T.timestamp_ms);
t = (t - t(1)) / 1000.0;

dt = diff(t);
dt_pos = dt(dt > 0);
fs_est = 1 / median(dt_pos);

fprintf("\n[Time]\n");
fprintf("Samples      : %d\n", height(T));
fprintf("Duration     : %.2f sec (%.2f min)\n", t(end), t(end)/60);
fprintf("Estimated Fs : %.2f Hz\n", fs_est);
fprintf("Median dt    : %.4f sec\n", median(dt_pos));
fprintf("Max dt       : %.4f sec\n", max(dt_pos));

gap_idx = find(diff(t) > 0.2);
fprintf("Large gaps > 0.2 sec: %d\n", numel(gap_idx));

if ~isempty(gap_idx)
    n_show = min(numel(gap_idx), 10);
    gap_time_before = t(gap_idx(1:n_show));
    gap_duration = t(gap_idx(1:n_show) + 1) - t(gap_idx(1:n_show));

    gap_table = table(gap_time_before(:), gap_duration(:), ...
        'VariableNames', {'time_before_gap_sec', 'gap_duration_sec'});
    fprintf("First few gaps:\n");
    disp(gap_table);
end

%% Required column check
required_cols = ["ekf_pwm_mean","M1","M2","M3","M4"];

for k = 1:numel(required_cols)
    if ~ismember(required_cols(k), T.Properties.VariableNames)
        error("Required column missing: %s", required_cols(k));
    end
end

%% Column selection
pwm_mean = double(T.ekf_pwm_mean);
pwm_bin = round(pwm_mean / pwm_round_step) * pwm_round_step;
pwm_bin(abs(pwm_bin - 1000) <= 10) = 1000;

if ismember("baro_rel_alt_m", T.Properties.VariableNames)
    baro_alt = double(T.baro_rel_alt_m);
    baro_name = "baro_rel_alt_m";
elseif ismember("baro_alt_unfiltered_m", T.Properties.VariableNames)
    baro_alt = double(T.baro_alt_unfiltered_m);
    baro_name = "baro_alt_unfiltered_m";
else
    error("No barometer altitude column found.");
end

has_raw_g     = ismember("raw_acc_norm_g", T.Properties.VariableNames);
has_acc_h     = ismember("acc_ned_h", T.Properties.VariableNames);
has_acc_d     = ismember("acc_ned_d", T.Properties.VariableNames);
has_gyro      = ismember("gyro_corrected_norm", T.Properties.VariableNames);
has_ekf_pos_d = ismember("ekf_pos_d", T.Properties.VariableNames);
has_ekf_vel_d = ismember("ekf_vel_d", T.Properties.VariableNames);
has_gnss_innov_d = ismember("ekf_innov_pos_d", T.Properties.VariableNames);
has_motor_on = ismember("motor_on_detected", T.Properties.VariableNames);
has_tether = ismember("tether_disturbance_candidate", T.Properties.VariableNames);

fprintf("\n[PWM bins detected]\n");
disp(sort(unique(pwm_bin)).');

%% Helper functions
rms_local = @(x) sqrt(mean(x.^2, 'omitnan'));
p95_abs = @(x) prctile(abs(x), 95);
range_local = @(x) max(x, [], 'omitnan') - min(x, [], 'omitnan');

%% Moving residuals for plots
baro_win = max(3, round(baro_residual_window_sec * fs_est));
baro_ma = movmean(baro_alt, baro_win, 'omitnan');
baro_res_ma = baro_alt - baro_ma;

if has_acc_d
    acc_d = double(T.acc_ned_d);
    acc_win = max(3, round(acc_residual_window_sec * fs_est));
    acc_d_ma = movmean(acc_d, acc_win, 'omitnan');
    acc_d_res_ma = acc_d - acc_d_ma;
end

if has_acc_h
    acc_h = double(T.acc_ned_h);
end

if has_raw_g
    raw_g = double(T.raw_acc_norm_g);
end

if has_gyro
    gyro_norm = double(T.gyro_corrected_norm);
end

%% Segment detection
change_idx = [1; find(diff(pwm_bin) ~= 0) + 1; numel(pwm_bin) + 1];

seg = struct([]);

for i = 1:numel(change_idx)-1
    i1 = change_idx(i);
    i2 = change_idx(i+1) - 1;

    this_pwm = pwm_bin(i1);
    if isnan(this_pwm)
        continue;
    end

    s = numel(seg) + 1;
    seg(s).seg_id = s;
    seg(s).pwm = this_pwm;
    seg(s).i1 = i1;
    seg(s).i2 = i2;
    seg(s).t_start = t(i1);
    seg(s).t_end = t(i2);
    seg(s).duration_sec = t(i2) - t(i1);
end

segments = struct2table(seg);

fprintf("\n[Detected PWM segments]\n");
disp(segments);

%% Segment summary
rows = struct([]);

for s = 1:height(segments)
    i1 = segments.i1(s);
    i2 = segments.i2(s);

    tt_start_used = segments.t_start(s) + ignore_segment_edge_sec;
    tt_end_used = segments.t_end(s) - ignore_segment_edge_sec;

    idx = (t >= tt_start_used) & (t <= tt_end_used);

    if sum(idx) < min_used_samples
        idx = false(size(t));
        idx(i1:i2) = true;
    end

    % target hold 구간은 중앙부 steady window도 별도 사용
    idx_steady = idx;
    if use_middle_ratio_for_hold && segments.pwm(s) == target_pwm
        dur = segments.t_end(s) - segments.t_start(s);
        margin = dur * (1.0 - hold_middle_ratio) / 2.0;
        t_mid1 = segments.t_start(s) + margin;
        t_mid2 = segments.t_end(s) - margin;
        idx_mid = (t >= t_mid1) & (t <= t_mid2);

        if sum(idx_mid) >= min_used_samples
            idx_steady = idx_mid;
        end
    end

    row.seg_id = segments.seg_id(s);
    row.pwm = segments.pwm(s);
    row.t_start = segments.t_start(s);
    row.t_end = segments.t_end(s);
    row.duration_sec = segments.duration_sec(s);
    row.used_samples = sum(idx);
    row.steady_used_samples = sum(idx_steady);
    row.short_segment = segments.duration_sec(s) < min_segment_sec || sum(idx) < min_used_samples;

    row.M1_mean = mean(T.M1(idx), 'omitnan');
    row.M2_mean = mean(T.M2(idx), 'omitnan');
    row.M3_mean = mean(T.M3(idx), 'omitnan');
    row.M4_mean = mean(T.M4(idx), 'omitnan');

    motor_mat = [T.M1(idx), T.M2(idx), T.M3(idx), T.M4(idx)];
    row.motor_pwm_spread_mean = mean(max(motor_mat, [], 2) - min(motor_mat, [], 2), 'omitnan');

    %% Baro full segment
    x = baro_alt(idx);
    tt = t(idx);
    baro_res_linear = local_linear_residual(tt, x);

    row.baro_mean_m = mean(x, 'omitnan');
    row.baro_raw_std_m = std(x, 'omitnan');
    row.baro_raw_range_m = range_local(x);
    row.baro_detrend_std_m = std(baro_res_linear, 'omitnan');
    row.baro_detrend_var_m2 = var(baro_res_linear, 'omitnan');
    row.baro_detrend_p95_m = p95_abs(baro_res_linear);
    row.baro_detrend_max_m = max(abs(baro_res_linear), [], 'omitnan');
    row.baro_movres_std_m = std(baro_res_ma(idx), 'omitnan');
    row.baro_movres_var_m2 = var(baro_res_ma(idx), 'omitnan');
    row.baro_movres_p95_m = p95_abs(baro_res_ma(idx));

    % drift slope
    [slope_baro, ~] = local_linear_fit(tt, x);
    row.baro_slope_mps = slope_baro;
    row.baro_slope_m_per_min = slope_baro * 60.0;

    %% Baro steady window
    x = baro_alt(idx_steady);
    tt = t(idx_steady);
    baro_res_steady = local_linear_residual(tt, x);

    row.steady_baro_detrend_std_m = std(baro_res_steady, 'omitnan');
    row.steady_baro_detrend_var_m2 = var(baro_res_steady, 'omitnan');
    row.steady_baro_detrend_p95_m = p95_abs(baro_res_steady);

    %% IMU
    if has_raw_g
        x = raw_g(idx);
        x_dev = x - mean(x, 'omitnan');
        row.raw_acc_norm_g_mean = mean(x, 'omitnan');
        row.raw_acc_norm_g_std = std(x, 'omitnan');
        row.raw_acc_norm_g_dev_p95 = p95_abs(x_dev);
        row.raw_acc_norm_g_dev_max = max(abs(x_dev), [], 'omitnan');

        xs = raw_g(idx_steady);
        xs_dev = xs - mean(xs, 'omitnan');
        row.steady_raw_acc_norm_g_std = std(xs, 'omitnan');
        row.steady_raw_acc_norm_g_dev_p95 = p95_abs(xs_dev);
    else
        row.raw_acc_norm_g_mean = NaN;
        row.raw_acc_norm_g_std = NaN;
        row.raw_acc_norm_g_dev_p95 = NaN;
        row.raw_acc_norm_g_dev_max = NaN;
        row.steady_raw_acc_norm_g_std = NaN;
        row.steady_raw_acc_norm_g_dev_p95 = NaN;
    end

    if has_acc_h
        x = acc_h(idx);
        row.acc_ned_h_mean = mean(x, 'omitnan');
        row.acc_ned_h_std = std(x, 'omitnan');
        row.acc_ned_h_rms = rms_local(x);
        row.acc_ned_h_p95 = p95_abs(x);
        row.acc_ned_h_max = max(abs(x), [], 'omitnan');

        xs = acc_h(idx_steady);
        row.steady_acc_ned_h_rms = rms_local(xs);
        row.steady_acc_ned_h_p95 = p95_abs(xs);
        row.steady_acc_ned_h_max = max(abs(xs), [], 'omitnan');
    else
        row.acc_ned_h_mean = NaN;
        row.acc_ned_h_std = NaN;
        row.acc_ned_h_rms = NaN;
        row.acc_ned_h_p95 = NaN;
        row.acc_ned_h_max = NaN;
        row.steady_acc_ned_h_rms = NaN;
        row.steady_acc_ned_h_p95 = NaN;
        row.steady_acc_ned_h_max = NaN;
    end

    if has_acc_d
        x = acc_d(idx);
        tt = t(idx);
        acc_d_res_linear = local_linear_residual(tt, x);

        row.acc_ned_d_mean = mean(x, 'omitnan');
        row.acc_ned_d_std = std(x, 'omitnan');
        row.acc_ned_d_detrend_std = std(acc_d_res_linear, 'omitnan');
        row.acc_ned_d_detrend_var = var(acc_d_res_linear, 'omitnan');
        row.acc_ned_d_movres_std = std(acc_d_res_ma(idx), 'omitnan');
        row.acc_ned_d_movres_var = var(acc_d_res_ma(idx), 'omitnan');
        row.acc_ned_d_p95 = p95_abs(x);
        row.acc_ned_d_max = max(abs(x), [], 'omitnan');

        xs = acc_d(idx_steady);
        tts = t(idx_steady);
        acc_d_res_steady = local_linear_residual(tts, xs);

        row.steady_acc_ned_d_detrend_std = std(acc_d_res_steady, 'omitnan');
        row.steady_acc_ned_d_detrend_var = var(acc_d_res_steady, 'omitnan');
        row.steady_acc_ned_d_p95 = p95_abs(xs);
        row.steady_acc_ned_d_max = max(abs(xs), [], 'omitnan');
    else
        row.acc_ned_d_mean = NaN;
        row.acc_ned_d_std = NaN;
        row.acc_ned_d_detrend_std = NaN;
        row.acc_ned_d_detrend_var = NaN;
        row.acc_ned_d_movres_std = NaN;
        row.acc_ned_d_movres_var = NaN;
        row.acc_ned_d_p95 = NaN;
        row.acc_ned_d_max = NaN;
        row.steady_acc_ned_d_detrend_std = NaN;
        row.steady_acc_ned_d_detrend_var = NaN;
        row.steady_acc_ned_d_p95 = NaN;
        row.steady_acc_ned_d_max = NaN;
    end

    if has_gyro
        x = gyro_norm(idx);
        row.gyro_norm_rms = rms_local(x);
        row.gyro_norm_p95 = p95_abs(x);
        row.gyro_norm_max = max(abs(x), [], 'omitnan');

        xs = gyro_norm(idx_steady);
        row.steady_gyro_norm_rms = rms_local(xs);
        row.steady_gyro_norm_p95 = p95_abs(xs);
        row.steady_gyro_norm_max = max(abs(xs), [], 'omitnan');
    else
        row.gyro_norm_rms = NaN;
        row.gyro_norm_p95 = NaN;
        row.gyro_norm_max = NaN;
        row.steady_gyro_norm_rms = NaN;
        row.steady_gyro_norm_p95 = NaN;
        row.steady_gyro_norm_max = NaN;
    end

    %% EKF
    if has_ekf_vel_d
        x = double(T.ekf_vel_d(idx));
        row.ekf_vel_d_mean = mean(x, 'omitnan');
        row.ekf_vel_d_std = std(x, 'omitnan');
        row.ekf_vel_d_p95 = p95_abs(x);
        row.ekf_vel_d_max = max(abs(x), [], 'omitnan');

        xs = double(T.ekf_vel_d(idx_steady));
        row.steady_ekf_vel_d_std = std(xs, 'omitnan');
        row.steady_ekf_vel_d_p95 = p95_abs(xs);
        row.steady_ekf_vel_d_max = max(abs(xs), [], 'omitnan');
    else
        row.ekf_vel_d_mean = NaN;
        row.ekf_vel_d_std = NaN;
        row.ekf_vel_d_p95 = NaN;
        row.ekf_vel_d_max = NaN;
        row.steady_ekf_vel_d_std = NaN;
        row.steady_ekf_vel_d_p95 = NaN;
        row.steady_ekf_vel_d_max = NaN;
    end

    if has_ekf_pos_d
        x = double(T.ekf_pos_d(idx));
        row.ekf_pos_d_mean = mean(x, 'omitnan');
        row.ekf_pos_d_std = std(x, 'omitnan');
        row.ekf_pos_d_range = range_local(x);
    else
        row.ekf_pos_d_mean = NaN;
        row.ekf_pos_d_std = NaN;
        row.ekf_pos_d_range = NaN;
    end

    if has_gnss_innov_d
        x = double(T.ekf_innov_pos_d(idx));
        row.gnss_d_innov_mean = mean(x, 'omitnan');
        row.gnss_d_innov_std = std(x, 'omitnan');
        row.gnss_d_innov_p95 = p95_abs(x);
        row.gnss_d_innov_max = max(abs(x), [], 'omitnan');
    else
        row.gnss_d_innov_mean = NaN;
        row.gnss_d_innov_std = NaN;
        row.gnss_d_innov_p95 = NaN;
        row.gnss_d_innov_max = NaN;
    end

    if has_motor_on
        row.motor_on_ratio = mean(double(T.motor_on_detected(idx)) > 0, 'omitnan');
    else
        row.motor_on_ratio = NaN;
    end

    if has_tether
        row.tether_disturbance_ratio = mean(double(T.tether_disturbance_candidate(idx)) > 0, 'omitnan');
    else
        row.tether_disturbance_ratio = NaN;
    end

    rows = [rows; row]; %#ok<AGROW>
end

summary = struct2table(rows);

fprintf("\n[Segment Summary]\n");
disp(summary);

%% Identify baseline / hold / recovery
target_rows = find(summary.pwm == target_pwm);
pre_base_rows = [];
post_base_rows = [];

if ~isempty(target_rows)
    target_seg = target_rows(1);

    pre_base_rows = find(summary.pwm(1:target_seg-1) == baseline_pwm);
    post_base_rows = find(summary.pwm(target_seg+1:end) == baseline_pwm) + target_seg;
end

fprintf("\n[Identified Test Structure]\n");

if isempty(target_rows)
    fprintf("Target PWM %d segment not found.\n", target_pwm);
else
    fprintf("Target PWM segment id: %d\n", summary.seg_id(target_seg));

    if ~isempty(pre_base_rows)
        fprintf("Pre-baseline segment id: %d\n", summary.seg_id(pre_base_rows(end)));
    else
        fprintf("Pre-baseline segment not found.\n");
    end

    if ~isempty(post_base_rows)
        fprintf("Post-recovery segment id: %d\n", summary.seg_id(post_base_rows(1)));
    else
        fprintf("Post-recovery segment not found.\n");
    end
end

%% Compare target hold with pre baseline and post recovery
compare = table();

if ~isempty(target_rows) && ~isempty(pre_base_rows)
    hold_idx = target_rows(1);
    base_idx = pre_base_rows(end);

    compare.metric = [
        "baro_detrend_var_m2";
        "baro_movres_var_m2";
        "steady_baro_detrend_var_m2";
        "acc_ned_d_detrend_var";
        "acc_ned_d_movres_var";
        "steady_acc_ned_d_detrend_var";
        "acc_ned_h_rms";
        "steady_acc_ned_h_rms";
        "raw_acc_norm_g_std";
        "gyro_norm_rms";
        "ekf_vel_d_p95";
        "steady_ekf_vel_d_p95"
    ];

    base_values = [
        summary.baro_detrend_var_m2(base_idx);
        summary.baro_movres_var_m2(base_idx);
        summary.steady_baro_detrend_var_m2(base_idx);
        summary.acc_ned_d_detrend_var(base_idx);
        summary.acc_ned_d_movres_var(base_idx);
        summary.steady_acc_ned_d_detrend_var(base_idx);
        summary.acc_ned_h_rms(base_idx);
        summary.steady_acc_ned_h_rms(base_idx);
        summary.raw_acc_norm_g_std(base_idx);
        summary.gyro_norm_rms(base_idx);
        summary.ekf_vel_d_p95(base_idx);
        summary.steady_ekf_vel_d_p95(base_idx)
    ];

    hold_values = [
        summary.baro_detrend_var_m2(hold_idx);
        summary.baro_movres_var_m2(hold_idx);
        summary.steady_baro_detrend_var_m2(hold_idx);
        summary.acc_ned_d_detrend_var(hold_idx);
        summary.acc_ned_d_movres_var(hold_idx);
        summary.steady_acc_ned_d_detrend_var(hold_idx);
        summary.acc_ned_h_rms(hold_idx);
        summary.steady_acc_ned_h_rms(hold_idx);
        summary.raw_acc_norm_g_std(hold_idx);
        summary.gyro_norm_rms(hold_idx);
        summary.ekf_vel_d_p95(hold_idx);
        summary.steady_ekf_vel_d_p95(hold_idx)
    ];

    compare.pre_baseline_value = base_values;
    compare.hold_value = hold_values;
    compare.hold_over_baseline_scale = arrayfun(@safe_ratio, hold_values, base_values);

    if ~isempty(post_base_rows)
        post_idx = post_base_rows(1);

        post_values = [
            summary.baro_detrend_var_m2(post_idx);
            summary.baro_movres_var_m2(post_idx);
            summary.steady_baro_detrend_var_m2(post_idx);
            summary.acc_ned_d_detrend_var(post_idx);
            summary.acc_ned_d_movres_var(post_idx);
            summary.steady_acc_ned_d_detrend_var(post_idx);
            summary.acc_ned_h_rms(post_idx);
            summary.steady_acc_ned_h_rms(post_idx);
            summary.raw_acc_norm_g_std(post_idx);
            summary.gyro_norm_rms(post_idx);
            summary.ekf_vel_d_p95(post_idx);
            summary.steady_ekf_vel_d_p95(post_idx)
        ];

        compare.post_recovery_value = post_values;
        compare.recovery_over_baseline_scale = arrayfun(@safe_ratio, post_values, base_values);
    end

    fprintf("\n[Hold vs Pre-Baseline Comparison]\n");
    disp(compare);
end

%% Conservative LUT decision
fprintf("\n[Conservative LUT Hint]\n");

if ~isempty(compare)
    r_baro_scale = compare.hold_over_baseline_scale(compare.metric == "steady_baro_detrend_var_m2");
    q_acc_scale = compare.hold_over_baseline_scale(compare.metric == "steady_acc_ned_d_detrend_var");
    acc_h_scale = compare.hold_over_baseline_scale(compare.metric == "steady_acc_ned_h_rms");

    R_baro_candidate = min(max(r_baro_scale, 1.0), 10.0);
    Q_acc_candidate = min(max(q_acc_scale, 1.0), 30.0);

    fprintf("Raw R_baro scale at PWM %d    : %.3f\n", target_pwm, r_baro_scale);
    fprintf("Clamped R_baro candidate      : %.3f\n", R_baro_candidate);
    fprintf("Raw Q_acc_z scale at PWM %d   : %.3f\n", target_pwm, q_acc_scale);
    fprintf("Clamped Q_acc_z candidate     : %.3f\n", Q_acc_candidate);
    fprintf("acc_ned_h RMS scale at PWM %d : %.3f\n", target_pwm, acc_h_scale);
else
    fprintf("Comparison not available.\n");
end

%% Save outputs
out_summary = "summary_4OUTDOOR_FIXED_PWM1300_HOLD_GROUND.csv";
out_compare = "compare_4OUTDOOR_FIXED_PWM1300_HOLD_GROUND.csv";

writetable(summary, out_summary);
fprintf("\nSaved:\n");
fprintf("  %s\n", out_summary);

if ~isempty(compare)
    writetable(compare, out_compare);
    fprintf("  %s\n", out_compare);
end

%% Plots 1: PWM and barometer
figure("Name","PWM1300 Hold - Barometer");
tiledlayout(4,1);

nexttile;
plot(t, T.M1, t, T.M2, t, T.M3, t, T.M4);
grid on;
ylabel("PWM");
title("Motor PWM");
legend("M1","M2","M3","M4");

nexttile;
plot(t, pwm_mean);
grid on;
ylabel("PWM");
title("ekf\_pwm\_mean");

nexttile;
plot(t, baro_alt);
grid on;
ylabel("m");
title(baro_name);

nexttile;
plot(t, baro_res_ma);
grid on;
ylabel("m");
xlabel("time [s]");
title(sprintf("%s residual from %.1f sec moving average", baro_name, baro_residual_window_sec));

%% Plots 2: IMU
figure("Name","PWM1300 Hold - IMU");
tiledlayout(4,1);

nexttile;
plot(t, pwm_mean);
grid on;
ylabel("PWM");
title("ekf\_pwm\_mean");

nexttile;
if has_raw_g
    plot(t, raw_g);
    grid on;
    ylabel("g");
    title("raw\_acc\_norm\_g");
end

nexttile;
if has_acc_h
    plot(t, acc_h);
    grid on;
    ylabel("m/s^2");
    title("acc\_ned\_h");
end

nexttile;
if has_acc_d
    plot(t, acc_d);
    hold on;
    plot(t, acc_d_res_ma);
    grid on;
    ylabel("m/s^2");
    xlabel("time [s]");
    title("acc\_ned\_d and moving residual");
    legend("acc\_ned\_d","moving residual");
end

%% Plots 3: EKF vertical
figure("Name","PWM1300 Hold - EKF Vertical");
tiledlayout(4,1);

nexttile;
plot(t, pwm_mean);
grid on;
ylabel("PWM");
title("ekf\_pwm\_mean");

nexttile;
if has_ekf_pos_d
    plot(t, T.ekf_pos_d);
    grid on;
    ylabel("m");
    title("ekf\_pos\_d");
end

nexttile;
if has_ekf_vel_d
    plot(t, T.ekf_vel_d);
    grid on;
    ylabel("m/s");
    title("ekf\_vel\_d");
end

nexttile;
if has_gnss_innov_d
    plot(t, T.ekf_innov_pos_d);
    grid on;
    ylabel("m");
    xlabel("time [s]");
    title("ekf\_innov\_pos\_d, GNSS D innovation");
end

%% Plots 4: Segment metrics
figure("Name","PWM1300 Hold - Segment Metrics");
tiledlayout(4,1);

nexttile;
plot(summary.seg_id, summary.baro_detrend_std_m, "-o");
grid on;
xlabel("Segment ID");
ylabel("m");
title("baro detrend std by segment");

nexttile;
plot(summary.seg_id, summary.acc_ned_d_detrend_std, "-o");
grid on;
xlabel("Segment ID");
ylabel("m/s^2");
title("acc\_ned\_d detrend std by segment");

nexttile;
plot(summary.seg_id, summary.acc_ned_h_rms, "-o");
grid on;
xlabel("Segment ID");
ylabel("m/s^2");
title("acc\_ned\_h RMS by segment");

nexttile;
plot(summary.seg_id, summary.ekf_vel_d_p95, "-o");
grid on;
xlabel("Segment ID");
ylabel("m/s");
title("ekf\_vel\_d P95 by segment");

%% Interpretation
fprintf("\n[How to read]\n");
fprintf("1) 1300 hold segment에서 acc_ned_d_detrend_var가 baseline보다 크면 Q_acc_z 증가 근거.\n");
fprintf("2) 1300 hold segment에서 baro_detrend_var_m2가 baseline보다 크면 R_baro 증가 근거.\n");
fprintf("3) post recovery가 baseline 근처로 돌아오면 PWM 영향이 일시적이라는 근거.\n");
fprintf("4) post recovery도 높으면 기체/바람/고정 상태/센서 drift 영향이 남은 것.\n");
fprintf("5) ekf_innov_pos_d는 GNSS D innovation이며 baro innovation이 아님.\n");

fprintf("\nDone.\n");

%% Local functions
function r = local_linear_residual(t, x)
    t = double(t(:));
    x = double(x(:));
    valid = isfinite(t) & isfinite(x);
    r = nan(size(x));

    if sum(valid) < 3
        r(valid) = x(valid) - mean(x(valid), 'omitnan');
        return;
    end

    p = polyfit(t(valid), x(valid), 1);
    trend = polyval(p, t(valid));
    r(valid) = x(valid) - trend;
end

function [slope, intercept] = local_linear_fit(t, x)
    t = double(t(:));
    x = double(x(:));
    valid = isfinite(t) & isfinite(x);

    if sum(valid) < 3
        slope = NaN;
        intercept = NaN;
        return;
    end

    p = polyfit(t(valid), x(valid), 1);
    slope = p(1);
    intercept = p(2);
end

function y = safe_ratio(a, b)
    if ~isfinite(a) || ~isfinite(b) || abs(b) < 1e-12
        y = NaN;
    else
        y = a / b;
    end
end