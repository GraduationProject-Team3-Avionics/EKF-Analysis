clear; clc; close all;

%% STRIX Fixed Ground PWM Repeat Analysis
% File:
%   3OUTDOOR_FIXED_PWM_REPEAT.CSV
%
% Purpose:
%   - 프롭 장착 / 실외 / 기체 고정 / PWM 반복 로그 분석
%   - 각 PWM 구간을 직전 1000 PWM recovery/baseline 구간과 비교
%   - Q_acc_z PWM scale 근거 확인
%   - R_baro PWM scale 근거 확인
%
% Important:
%   - ekf_innov_pos_d는 baro innovation이 아니라 GNSS D position innovation으로 해석
%   - baro innovation = baro_alt - height_pred 는 아직 별도 로깅 필요
%   - 1400 이상 위험 구간은 통계용보다 event 확인용

%% User settings
csv_file = "data/3OUTDOOR_FIXED_PWM_REPEAT.CSV";

if ~isfile(csv_file)
    csv_file = "3OUTDOOR_FIXED_PWM_REPEAT.CSV";
end

pwm_round_step = 50;
baseline_pwm = 1000;

% 구간 전환 직후/직전 과도응답 제거
ignore_segment_edge_sec = 2.0;

% 너무 짧은 구간 경고 기준
min_segment_sec = 8.0;
min_used_samples = 30;

% moving residual은 plot용
baro_residual_window_sec = 5.0;
acc_residual_window_sec  = 2.0;

% PWM scale 계산에서 제외할 PWM
exclude_short_for_scale = true;

%% Load
T = readtable(csv_file);

fprintf("\n========================================\n");
fprintf("STRIX Fixed Ground PWM Repeat Analysis\n");
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
    gap_duration    = t(gap_idx(1:n_show) + 1) - t(gap_idx(1:n_show));

    gap_table = table(gap_time_before(:), gap_duration(:), ...
        'VariableNames', {'time_before_gap_sec', 'gap_duration_sec'});

    fprintf("First few gaps:\n");
    disp(gap_table);
end

%% Required columns
required_cols = ["ekf_pwm_mean", "M1", "M2", "M3", "M4"];

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

has_pressure = ismember("baro_pressure_pa", T.Properties.VariableNames);
has_raw_g    = ismember("raw_acc_norm_g", T.Properties.VariableNames);
has_acc_h    = ismember("acc_ned_h", T.Properties.VariableNames);
has_acc_d    = ismember("acc_ned_d", T.Properties.VariableNames);
has_gyro     = ismember("gyro_corrected_norm", T.Properties.VariableNames);
has_ekf_vel_d = ismember("ekf_vel_d", T.Properties.VariableNames);
has_ekf_pos_d = ismember("ekf_pos_d", T.Properties.VariableNames);
has_motor_on = ismember("motor_on_detected", T.Properties.VariableNames);
has_tether   = ismember("tether_disturbance_candidate", T.Properties.VariableNames);

fprintf("\n[PWM bins detected]\n");
disp(sort(unique(pwm_bin)).');

%% Helper functions
rms_local = @(x) sqrt(mean(x.^2, 'omitnan'));
p95_abs = @(x) prctile(abs(x), 95);
range_local = @(x) max(x, [], 'omitnan') - min(x, [], 'omitnan');

linear_residual = @(tt, x) local_linear_residual(tt, x);

%% Whole-signal residuals for plots
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
    tt_end_used   = segments.t_end(s)   - ignore_segment_edge_sec;

    idx = (t >= tt_start_used) & (t <= tt_end_used);

    % 구간이 너무 짧으면 edge 제거 없이 사용
    if sum(idx) < min_used_samples
        idx = false(size(t));
        idx(i1:i2) = true;
    end

    tt = t(idx);

    row.seg_id = segments.seg_id(s);
    row.pwm = segments.pwm(s);
    row.t_start = segments.t_start(s);
    row.t_end = segments.t_end(s);
    row.duration_sec = segments.duration_sec(s);
    row.used_samples = sum(idx);
    row.short_segment = segments.duration_sec(s) < min_segment_sec || sum(idx) < min_used_samples;

    row.M1_mean = mean(T.M1(idx), 'omitnan');
    row.M2_mean = mean(T.M2(idx), 'omitnan');
    row.M3_mean = mean(T.M3(idx), 'omitnan');
    row.M4_mean = mean(T.M4(idx), 'omitnan');
    row.motor_pwm_spread_mean = mean(max([T.M1(idx), T.M2(idx), T.M3(idx), T.M4(idx)], [], 2) ...
                                   - min([T.M1(idx), T.M2(idx), T.M3(idx), T.M4(idx)], [], 2), 'omitnan');

    %% Barometer
    x = baro_alt(idx);
    baro_res_linear = linear_residual(tt, x);

    row.baro_mean_m = mean(x, 'omitnan');
    row.baro_raw_std_m = std(x, 'omitnan');
    row.baro_raw_range_m = range_local(x);
    row.baro_detrend_std_m = std(baro_res_linear, 'omitnan');
    row.baro_detrend_var_m2 = var(baro_res_linear, 'omitnan');
    row.baro_detrend_p95_m = p95_abs(baro_res_linear);
    row.baro_detrend_max_m = max(abs(baro_res_linear), [], 'omitnan');

    x_ma_res = baro_res_ma(idx);
    row.baro_movres_std_m = std(x_ma_res, 'omitnan');
    row.baro_movres_var_m2 = var(x_ma_res, 'omitnan');
    row.baro_movres_p95_m = p95_abs(x_ma_res);

    if has_pressure
        pr = double(T.baro_pressure_pa(idx));
        pr_res = linear_residual(tt, pr);
        row.pressure_detrend_std_pa = std(pr_res, 'omitnan');
        row.pressure_detrend_p95_pa = p95_abs(pr_res);
    else
        row.pressure_detrend_std_pa = NaN;
        row.pressure_detrend_p95_pa = NaN;
    end

    %% IMU
    if has_raw_g
        x = raw_g(idx);
        x_dev = x - mean(x, 'omitnan');

        row.raw_acc_norm_g_mean = mean(x, 'omitnan');
        row.raw_acc_norm_g_std = std(x, 'omitnan');
        row.raw_acc_norm_g_dev_p95 = p95_abs(x_dev);
        row.raw_acc_norm_g_dev_max = max(abs(x_dev), [], 'omitnan');
    else
        row.raw_acc_norm_g_mean = NaN;
        row.raw_acc_norm_g_std = NaN;
        row.raw_acc_norm_g_dev_p95 = NaN;
        row.raw_acc_norm_g_dev_max = NaN;
    end

    if has_acc_h
        x = acc_h(idx);

        row.acc_ned_h_mean = mean(x, 'omitnan');
        row.acc_ned_h_std = std(x, 'omitnan');
        row.acc_ned_h_rms = rms_local(x);
        row.acc_ned_h_p95 = p95_abs(x);
        row.acc_ned_h_max = max(abs(x), [], 'omitnan');
    else
        row.acc_ned_h_mean = NaN;
        row.acc_ned_h_std = NaN;
        row.acc_ned_h_rms = NaN;
        row.acc_ned_h_p95 = NaN;
        row.acc_ned_h_max = NaN;
    end

    if has_acc_d
        x = acc_d(idx);
        x_res_linear = linear_residual(tt, x);
        x_res_ma = acc_d_res_ma(idx);

        row.acc_ned_d_mean = mean(x, 'omitnan');
        row.acc_ned_d_std = std(x, 'omitnan');
        row.acc_ned_d_detrend_std = std(x_res_linear, 'omitnan');
        row.acc_ned_d_detrend_var = var(x_res_linear, 'omitnan');
        row.acc_ned_d_movres_std = std(x_res_ma, 'omitnan');
        row.acc_ned_d_movres_var = var(x_res_ma, 'omitnan');
        row.acc_ned_d_p95 = p95_abs(x);
        row.acc_ned_d_max = max(abs(x), [], 'omitnan');
    else
        row.acc_ned_d_mean = NaN;
        row.acc_ned_d_std = NaN;
        row.acc_ned_d_detrend_std = NaN;
        row.acc_ned_d_detrend_var = NaN;
        row.acc_ned_d_movres_std = NaN;
        row.acc_ned_d_movres_var = NaN;
        row.acc_ned_d_p95 = NaN;
        row.acc_ned_d_max = NaN;
    end

    if has_gyro
        x = gyro_norm(idx);

        row.gyro_norm_mean = mean(x, 'omitnan');
        row.gyro_norm_std = std(x, 'omitnan');
        row.gyro_norm_rms = rms_local(x);
        row.gyro_norm_p95 = p95_abs(x);
        row.gyro_norm_max = max(abs(x), [], 'omitnan');
    else
        row.gyro_norm_mean = NaN;
        row.gyro_norm_std = NaN;
        row.gyro_norm_rms = NaN;
        row.gyro_norm_p95 = NaN;
        row.gyro_norm_max = NaN;
    end

    %% EKF vertical
    if has_ekf_vel_d
        x = double(T.ekf_vel_d(idx));

        row.ekf_vel_d_mean = mean(x, 'omitnan');
        row.ekf_vel_d_std = std(x, 'omitnan');
        row.ekf_vel_d_p95 = p95_abs(x);
        row.ekf_vel_d_max = max(abs(x), [], 'omitnan');
    else
        row.ekf_vel_d_mean = NaN;
        row.ekf_vel_d_std = NaN;
        row.ekf_vel_d_p95 = NaN;
        row.ekf_vel_d_max = NaN;
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

    if ismember("ekf_innov_pos_d", T.Properties.VariableNames)
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
        x = double(T.motor_on_detected(idx));
        row.motor_on_ratio = mean(x > 0, 'omitnan');
    else
        row.motor_on_ratio = NaN;
    end

    if has_tether
        x = double(T.tether_disturbance_candidate(idx));
        row.tether_disturbance_ratio = mean(x > 0, 'omitnan');
    else
        row.tether_disturbance_ratio = NaN;
    end

    rows = [rows; row]; %#ok<AGROW>
end

summary = struct2table(rows);

fprintf("\n[Segment Summary]\n");
disp(summary);

%% Local baseline comparison
% 각 motor PWM segment를 직전 1000 PWM segment와 비교
comp_rows = struct([]);

for s = 1:height(summary)
    if summary.pwm(s) == baseline_pwm
        continue;
    end

    prev_base = find(summary.pwm(1:s-1) == baseline_pwm, 1, 'last');

    if isempty(prev_base)
        continue;
    end

    if exclude_short_for_scale && summary.short_segment(s)
        use_for_scale = false;
    else
        use_for_scale = true;
    end

    r.seg_id = summary.seg_id(s);
    r.pwm = summary.pwm(s);
    r.base_seg_id = summary.seg_id(prev_base);
    r.base_duration_sec = summary.duration_sec(prev_base);
    r.duration_sec = summary.duration_sec(s);
    r.short_segment = summary.short_segment(s);
    r.use_for_scale = use_for_scale;

    r.R_baro_scale_detrend = safe_ratio(summary.baro_detrend_var_m2(s), summary.baro_detrend_var_m2(prev_base));
    r.R_baro_scale_movres  = safe_ratio(summary.baro_movres_var_m2(s), summary.baro_movres_var_m2(prev_base));

    r.Q_acc_z_scale_detrend = safe_ratio(summary.acc_ned_d_detrend_var(s), summary.acc_ned_d_detrend_var(prev_base));
    r.Q_acc_z_scale_movres  = safe_ratio(summary.acc_ned_d_movres_var(s), summary.acc_ned_d_movres_var(prev_base));

    r.acc_h_rms_scale = safe_ratio(summary.acc_ned_h_rms(s), summary.acc_ned_h_rms(prev_base));
    r.raw_g_std_scale = safe_ratio(summary.raw_acc_norm_g_std(s), summary.raw_acc_norm_g_std(prev_base));
    r.gyro_rms_scale  = safe_ratio(summary.gyro_norm_rms(s), summary.gyro_norm_rms(prev_base));
    r.ekf_vel_d_p95_scale = safe_ratio(summary.ekf_vel_d_p95(s), summary.ekf_vel_d_p95(prev_base));

    r.baro_std_m = summary.baro_detrend_std_m(s);
    r.base_baro_std_m = summary.baro_detrend_std_m(prev_base);

    r.acc_d_std = summary.acc_ned_d_detrend_std(s);
    r.base_acc_d_std = summary.acc_ned_d_detrend_std(prev_base);

    r.acc_h_rms = summary.acc_ned_h_rms(s);
    r.base_acc_h_rms = summary.acc_ned_h_rms(prev_base);

    comp_rows = [comp_rows; r]; %#ok<AGROW>
end

if ~isempty(comp_rows)
    local_compare = struct2table(comp_rows);
else
    local_compare = table();
end

fprintf("\n[Local Baseline Comparison]\n");
disp(local_compare);

%% Group by PWM using only non-short motor segments
if ~isempty(local_compare)
    pwm_values = sort(unique(local_compare.pwm));
    g_rows = struct([]);

    for k = 1:numel(pwm_values)
        p = pwm_values(k);
        idx = local_compare.pwm == p & local_compare.use_for_scale;

        if ~any(idx)
            idx = local_compare.pwm == p;
        end

        g.pwm = p;
        g.n_segments = sum(idx);
        g.total_duration_sec = sum(local_compare.duration_sec(idx));

        g.R_baro_scale_detrend_median = median(local_compare.R_baro_scale_detrend(idx), 'omitnan');
        g.R_baro_scale_detrend_mean   = mean(local_compare.R_baro_scale_detrend(idx), 'omitnan');
        g.R_baro_scale_movres_median  = median(local_compare.R_baro_scale_movres(idx), 'omitnan');
        g.R_baro_scale_movres_mean    = mean(local_compare.R_baro_scale_movres(idx), 'omitnan');

        g.Q_acc_z_scale_detrend_median = median(local_compare.Q_acc_z_scale_detrend(idx), 'omitnan');
        g.Q_acc_z_scale_detrend_mean   = mean(local_compare.Q_acc_z_scale_detrend(idx), 'omitnan');
        g.Q_acc_z_scale_movres_median  = median(local_compare.Q_acc_z_scale_movres(idx), 'omitnan');
        g.Q_acc_z_scale_movres_mean    = mean(local_compare.Q_acc_z_scale_movres(idx), 'omitnan');

        g.acc_h_rms_scale_median = median(local_compare.acc_h_rms_scale(idx), 'omitnan');
        g.raw_g_std_scale_median = median(local_compare.raw_g_std_scale(idx), 'omitnan');
        g.gyro_rms_scale_median  = median(local_compare.gyro_rms_scale(idx), 'omitnan');
        g.ekf_vel_d_p95_scale_median = median(local_compare.ekf_vel_d_p95_scale(idx), 'omitnan');

        g.baro_std_m_median = median(local_compare.baro_std_m(idx), 'omitnan');
        g.acc_d_std_median  = median(local_compare.acc_d_std(idx), 'omitnan');
        g.acc_h_rms_median  = median(local_compare.acc_h_rms(idx), 'omitnan');

        g_rows = [g_rows; g]; %#ok<AGROW>
    end

    grouped = struct2table(g_rows);
else
    grouped = table();
end

fprintf("\n[PWM Grouped Local-Comparison Summary]\n");
disp(grouped);

%% Conservative LUT candidate
% raw ratio는 너무 커질 수 있으므로 clamp한 후보도 같이 출력
if ~isempty(grouped)
    lut = table();
    lut.pwm = grouped.pwm;

    % Baro R은 아직 근거가 약하므로 최소 1 이상, 최대 10 정도로 제한
    lut.R_baro_scale_raw = grouped.R_baro_scale_detrend_median;
    lut.R_baro_scale_candidate = min(max(grouped.R_baro_scale_detrend_median, 1.0), 10.0);

    % Acc Q는 ratio가 매우 커질 수 있으므로 최대 30 정도로 제한
    lut.Q_acc_z_scale_raw = grouped.Q_acc_z_scale_detrend_median;
    lut.Q_acc_z_scale_candidate = min(max(grouped.Q_acc_z_scale_detrend_median, 1.0), 30.0);

    fprintf("\n[Conservative LUT Candidate]\n");
    disp(lut);
end

%% Save outputs
out_summary = "summary_3OUTDOOR_FIXED_PWM_REPEAT.csv";
out_compare = "local_compare_3OUTDOOR_FIXED_PWM_REPEAT.csv";
out_grouped = "grouped_3OUTDOOR_FIXED_PWM_REPEAT.csv";
out_lut = "lut_candidate_3OUTDOOR_FIXED_PWM_REPEAT.csv";

writetable(summary, out_summary);

if ~isempty(local_compare)
    writetable(local_compare, out_compare);
end

if ~isempty(grouped)
    writetable(grouped, out_grouped);
end

if exist("lut", "var")
    writetable(lut, out_lut);
end

fprintf("\nSaved:\n");
fprintf("  %s\n", out_summary);
if ~isempty(local_compare), fprintf("  %s\n", out_compare); end
if ~isempty(grouped), fprintf("  %s\n", out_grouped); end
if exist("lut", "var"), fprintf("  %s\n", out_lut); end

%% Plots 1: PWM and baro
figure("Name","PWM Repeat - Barometer Overview");
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
figure("Name","PWM Repeat - IMU Response");
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
    legend("acc\_ned\_d", "moving residual");
end

%% Plots 3: EKF vertical
figure("Name","PWM Repeat - EKF Vertical");
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
if ismember("ekf_innov_pos_d", T.Properties.VariableNames)
    plot(t, T.ekf_innov_pos_d);
    grid on;
    ylabel("m");
    xlabel("time [s]");
    title("ekf\_innov\_pos\_d, GNSS D innovation");
end

%% Plots 4: grouped local comparison
if ~isempty(grouped)
    figure("Name","PWM Repeat - Local Baseline Scale");
    tiledlayout(4,1);

    nexttile;
    plot(grouped.pwm, grouped.R_baro_scale_detrend_median, "-o");
    grid on;
    xlabel("PWM");
    ylabel("scale");
    title("R\_baro scale candidate, local baseline detrend");

    nexttile;
    plot(grouped.pwm, grouped.Q_acc_z_scale_detrend_median, "-o");
    grid on;
    xlabel("PWM");
    ylabel("scale");
    title("Q\_acc\_z scale candidate, local baseline detrend");

    nexttile;
    plot(grouped.pwm, grouped.acc_d_std_median, "-o");
    grid on;
    xlabel("PWM");
    ylabel("m/s^2");
    title("acc\_ned\_d detrend std by PWM");

    nexttile;
    plot(grouped.pwm, grouped.acc_h_rms_median, "-o");
    grid on;
    xlabel("PWM");
    ylabel("m/s^2");
    title("acc\_ned\_h RMS by PWM");
end

%% Plots 5: conservative LUT
if exist("lut", "var")
    figure("Name","PWM Repeat - Conservative LUT Candidate");
    tiledlayout(2,1);

    nexttile;
    plot(lut.pwm, lut.R_baro_scale_candidate, "-o");
    grid on;
    xlabel("PWM");
    ylabel("scale");
    title("Conservative R\_baro scale candidate");

    nexttile;
    plot(lut.pwm, lut.Q_acc_z_scale_candidate, "-o");
    grid on;
    xlabel("PWM");
    ylabel("scale");
    title("Conservative Q\_acc\_z scale candidate");
end

%% Interpretation guide
fprintf("\n[How to read]\n");
fprintf("1) Segment Summary: 각 PWM 구간 자체의 raw metric 확인.\n");
fprintf("2) Local Baseline Comparison: 각 PWM 구간을 직전 1000 PWM 구간과 비교.\n");
fprintf("3) R_baro_scale이 1보다 크면 해당 PWM에서 baro noise 증가 근거.\n");
fprintf("4) Q_acc_z_scale이 1보다 크면 해당 PWM에서 vertical acceleration noise 증가 근거.\n");
fprintf("5) Conservative LUT Candidate는 raw ratio를 그대로 쓰지 않고 clamp한 초기 구현 후보.\n");
fprintf("6) 1400 이상 짧은 구간은 short_segment=true이면 통계 확정용에서 제외.\n");
fprintf("7) ekf_innov_pos_d는 GNSS D innovation이지 baro innovation이 아님.\n");

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

function y = safe_ratio(a, b)
    if ~isfinite(a) || ~isfinite(b) || abs(b) < 1e-12
        y = NaN;
    else
        y = a / b;
    end
end