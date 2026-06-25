clear; clc; close all;

%% STRIX Fixed Ground PWM Sweep Analysis
% File:
%   2OUTDOOR_FIXED_PWM_SWEEP_GROUND.CSV
%
% Purpose:
%   - 프롭 장착 / 실외 / 기체 고정 / PWM sweep 로그 분석
%   - PWM 증가에 따른 barometer residual, IMU vibration, EKF vertical state 변화 확인
%   - Adaptive R_baro PWM scale, Q_acc_z PWM scale 설계용 1차 데이터 추출
%
% Notes:
%   - 1400 PWM 구간이 짧으면 통계 신뢰도 낮음
%   - ekf_innov_pos_d는 baro innovation이 아니라 GNSS D position innovation으로 해석해야 함

%% User settings
csv_file = "data/2OUTDOOR_FIXED_PWM_SWEEP_GROUND.CSV";

% PWM 구간 판단용
pwm_round_step = 50;          % PWM을 50 단위로 반올림
min_segment_sec = 5.0;        % 이보다 짧은 구간은 summary에서 warning
ignore_segment_edge_sec = 2.0; % PWM 전환 직후/직전 과도응답 제거

% residual 계산용 moving average window
baro_residual_window_sec = 5.0;
acc_residual_window_sec  = 2.0;

% 비교 기준 PWM
baseline_pwm = 1000;

%% Load
T = readtable(csv_file);

fprintf("\n========================================\n");
fprintf("STRIX Fixed Ground PWM Sweep Analysis\n");
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

%% Required columns check
required_cols = ["ekf_pwm_mean", "M1", "M2", "M3", "M4"];
for k = 1:numel(required_cols)
    if ~ismember(required_cols(k), T.Properties.VariableNames)
        error("Required column missing: %s", required_cols(k));
    end
end

%% PWM binning
pwm_mean = double(T.ekf_pwm_mean);
pwm_bin = round(pwm_mean / pwm_round_step) * pwm_round_step;

% 너무 작은 흔들림은 1000으로 정리
pwm_bin(abs(pwm_bin - 1000) <= 10) = 1000;

unique_pwm = unique(pwm_bin);
unique_pwm = unique_pwm(~isnan(unique_pwm));
unique_pwm = sort(unique_pwm);

fprintf("\n[PWM bins detected]\n");
disp(unique_pwm.');

%% Segment detection
% 연속된 동일 PWM bin 구간 찾기
change_idx = [1; find(diff(pwm_bin) ~= 0) + 1; numel(pwm_bin) + 1];

segments = table();
seg_count = 0;

for i = 1:numel(change_idx)-1
    i1 = change_idx(i);
    i2 = change_idx(i+1) - 1;

    this_pwm = pwm_bin(i1);
    this_duration = t(i2) - t(i1);

    if isnan(this_pwm)
        continue;
    end

    seg_count = seg_count + 1;
    segments.seg_id(seg_count,1) = seg_count;
    segments.pwm(seg_count,1) = this_pwm;
    segments.i1(seg_count,1) = i1;
    segments.i2(seg_count,1) = i2;
    segments.t_start(seg_count,1) = t(i1);
    segments.t_end(seg_count,1) = t(i2);
    segments.duration_sec(seg_count,1) = this_duration;
end

fprintf("\n[Detected PWM segments]\n");
disp(segments);

%% Helper functions
rms_local = @(x) sqrt(mean(x.^2, 'omitnan'));
p95_abs = @(x) prctile(abs(x), 95);
range_local = @(x) max(x, [], 'omitnan') - min(x, [], 'omitnan');

%% Pick barometer column
if ismember("baro_rel_alt_m", T.Properties.VariableNames)
    baro_alt = double(T.baro_rel_alt_m);
    baro_name = "baro_rel_alt_m";
elseif ismember("baro_alt_unfiltered_m", T.Properties.VariableNames)
    baro_alt = double(T.baro_alt_unfiltered_m);
    baro_name = "baro_alt_unfiltered_m";
else
    error("No barometer altitude column found.");
end

% moving average residual
baro_win = max(3, round(baro_residual_window_sec * fs_est));
baro_ma = movmean(baro_alt, baro_win, 'omitnan');
baro_res_ma = baro_alt - baro_ma;

% pressure residual도 있으면 계산
has_pressure = ismember("baro_pressure_pa", T.Properties.VariableNames);
if has_pressure
    pressure = double(T.baro_pressure_pa);
    pressure_ma = movmean(pressure, baro_win, 'omitnan');
    pressure_res_ma = pressure - pressure_ma;
end

%% Acceleration residuals
acc_win = max(3, round(acc_residual_window_sec * fs_est));

has_acc_d = ismember("acc_ned_d", T.Properties.VariableNames);
if has_acc_d
    acc_d = double(T.acc_ned_d);
    acc_d_ma = movmean(acc_d, acc_win, 'omitnan');
    acc_d_res = acc_d - acc_d_ma;
end

has_acc_h = ismember("acc_ned_h", T.Properties.VariableNames);
if has_acc_h
    acc_h = double(T.acc_ned_h);
end

has_raw_g = ismember("raw_acc_norm_g", T.Properties.VariableNames);
if has_raw_g
    raw_g = double(T.raw_acc_norm_g);
end

has_gyro = ismember("gyro_corrected_norm", T.Properties.VariableNames);
if has_gyro
    gyro_norm = double(T.gyro_corrected_norm);
end

%% Segment summary
summary = table();

for s = 1:height(segments)
    i1 = segments.i1(s);
    i2 = segments.i2(s);

    tt1 = segments.t_start(s) + ignore_segment_edge_sec;
    tt2 = segments.t_end(s)   - ignore_segment_edge_sec;

    idx = (t >= tt1) & (t <= tt2);

    % 구간이 너무 짧으면 edge 제거 없이 사용
    if sum(idx) < 10
        idx = false(size(t));
        idx(i1:i2) = true;
    end

    x_baro = baro_alt(idx);
    x_baro_res = baro_res_ma(idx);

    summary.seg_id(s,1) = segments.seg_id(s);
    summary.pwm(s,1) = segments.pwm(s);
    summary.t_start(s,1) = segments.t_start(s);
    summary.t_end(s,1) = segments.t_end(s);
    summary.duration_sec(s,1) = segments.duration_sec(s);
    summary.used_samples(s,1) = sum(idx);
    summary.short_segment(s,1) = segments.duration_sec(s) < min_segment_sec;

    summary.baro_mean_m(s,1) = mean(x_baro, 'omitnan');
    summary.baro_raw_std_m(s,1) = std(x_baro, 'omitnan');
    summary.baro_raw_range_m(s,1) = range_local(x_baro);
    summary.baro_res_std_m(s,1) = std(x_baro_res, 'omitnan');
    summary.baro_res_var_m2(s,1) = var(x_baro_res, 'omitnan');
    summary.baro_res_p95_m(s,1) = p95_abs(x_baro_res);
    summary.baro_res_max_m(s,1) = max(abs(x_baro_res), [], 'omitnan');

    if has_pressure
        x_pr = pressure_res_ma(idx);
        summary.pressure_res_std_pa(s,1) = std(x_pr, 'omitnan');
        summary.pressure_res_p95_pa(s,1) = p95_abs(x_pr);
    end

    if has_raw_g
        x = raw_g(idx);
        summary.raw_acc_norm_g_mean(s,1) = mean(x, 'omitnan');
        summary.raw_acc_norm_g_std(s,1) = std(x, 'omitnan');
        summary.raw_acc_norm_g_p95(s,1) = p95_abs(x - mean(x,'omitnan'));
        summary.raw_acc_norm_g_maxdev(s,1) = max(abs(x - mean(x,'omitnan')), [], 'omitnan');
    end

    if has_acc_h
        x = acc_h(idx);
        summary.acc_ned_h_rms(s,1) = rms_local(x);
        summary.acc_ned_h_p95(s,1) = p95_abs(x);
        summary.acc_ned_h_max(s,1) = max(abs(x), [], 'omitnan');
    end

    if has_acc_d
        x = acc_d(idx);
        xr = acc_d_res(idx);
        summary.acc_ned_d_mean(s,1) = mean(x, 'omitnan');
        summary.acc_ned_d_std(s,1) = std(x, 'omitnan');
        summary.acc_ned_d_res_std(s,1) = std(xr, 'omitnan');
        summary.acc_ned_d_res_var(s,1) = var(xr, 'omitnan');
        summary.acc_ned_d_p95(s,1) = p95_abs(x);
        summary.acc_ned_d_max(s,1) = max(abs(x), [], 'omitnan');
    end

    if has_gyro
        x = gyro_norm(idx);
        summary.gyro_norm_rms(s,1) = rms_local(x);
        summary.gyro_norm_p95(s,1) = p95_abs(x);
        summary.gyro_norm_max(s,1) = max(abs(x), [], 'omitnan');
    end

    if ismember("ekf_vel_d", T.Properties.VariableNames)
        x = double(T.ekf_vel_d(idx));
        summary.ekf_vel_d_mean(s,1) = mean(x, 'omitnan');
        summary.ekf_vel_d_std(s,1) = std(x, 'omitnan');
        summary.ekf_vel_d_p95(s,1) = p95_abs(x);
        summary.ekf_vel_d_max(s,1) = max(abs(x), [], 'omitnan');
    end

    if ismember("ekf_pos_d", T.Properties.VariableNames)
        x = double(T.ekf_pos_d(idx));
        summary.ekf_pos_d_std(s,1) = std(x, 'omitnan');
        summary.ekf_pos_d_range(s,1) = range_local(x);
    end

    if ismember("tether_disturbance_candidate", T.Properties.VariableNames)
        x = double(T.tether_disturbance_candidate(idx));
        summary.tether_disturbance_ratio(s,1) = mean(x > 0, 'omitnan');
    end

    if ismember("motor_on_detected", T.Properties.VariableNames)
        x = double(T.motor_on_detected(idx));
        summary.motor_on_ratio(s,1) = mean(x > 0, 'omitnan');
    end
end

fprintf("\n[Segment Summary]\n");
disp(summary);

%% PWM grouped summary
% 같은 PWM이 여러 segment로 나뉘었을 때 평균
pwm_values = unique(summary.pwm);
grouped = table();

for k = 1:numel(pwm_values)
    p = pwm_values(k);
    rows = summary.pwm == p;

    grouped.pwm(k,1) = p;
    grouped.total_duration_sec(k,1) = sum(summary.duration_sec(rows));
    grouped.total_used_samples(k,1) = sum(summary.used_samples(rows));

    grouped.baro_res_std_m_mean(k,1) = mean(summary.baro_res_std_m(rows), 'omitnan');
    grouped.baro_res_var_m2_mean(k,1) = mean(summary.baro_res_var_m2(rows), 'omitnan');
    grouped.baro_res_p95_m_mean(k,1) = mean(summary.baro_res_p95_m(rows), 'omitnan');

    if ismember("acc_ned_d_res_var", summary.Properties.VariableNames)
        grouped.acc_ned_d_res_var_mean(k,1) = mean(summary.acc_ned_d_res_var(rows), 'omitnan');
        grouped.acc_ned_d_res_std_mean(k,1) = mean(summary.acc_ned_d_res_std(rows), 'omitnan');
    end

    if ismember("acc_ned_h_rms", summary.Properties.VariableNames)
        grouped.acc_ned_h_rms_mean(k,1) = mean(summary.acc_ned_h_rms(rows), 'omitnan');
        grouped.acc_ned_h_p95_mean(k,1) = mean(summary.acc_ned_h_p95(rows), 'omitnan');
    end

    if ismember("raw_acc_norm_g_std", summary.Properties.VariableNames)
        grouped.raw_acc_norm_g_std_mean(k,1) = mean(summary.raw_acc_norm_g_std(rows), 'omitnan');
    end

    if ismember("gyro_norm_rms", summary.Properties.VariableNames)
        grouped.gyro_norm_rms_mean(k,1) = mean(summary.gyro_norm_rms(rows), 'omitnan');
    end

    if ismember("ekf_vel_d_p95", summary.Properties.VariableNames)
        grouped.ekf_vel_d_p95_mean(k,1) = mean(summary.ekf_vel_d_p95(rows), 'omitnan');
        grouped.ekf_vel_d_max_mean(k,1) = mean(summary.ekf_vel_d_max(rows), 'omitnan');
    end
end

fprintf("\n[PWM Grouped Summary]\n");
disp(grouped);

%% Scale candidates relative to baseline PWM
base_row = grouped.pwm == baseline_pwm;

if any(base_row)
    R_base = grouped.baro_res_var_m2_mean(base_row);

    grouped.R_baro_pwm_scale_candidate = grouped.baro_res_var_m2_mean / R_base;

    if ismember("acc_ned_d_res_var_mean", grouped.Properties.VariableNames)
        Q_base = grouped.acc_ned_d_res_var_mean(base_row);
        grouped.Q_acc_z_pwm_scale_candidate = grouped.acc_ned_d_res_var_mean / Q_base;
    end

    fprintf("\n[Adaptive LUT scale candidates relative to PWM %d]\n", baseline_pwm);
    disp(grouped(:, contains(grouped.Properties.VariableNames, "pwm") | ...
                    contains(grouped.Properties.VariableNames, "scale") | ...
                    contains(grouped.Properties.VariableNames, "duration")));
else
    warning("Baseline PWM %d not found. Scale candidates not computed.", baseline_pwm);
end

%% Save summary
out_summary = "summary_2OUTDOOR_FIXED_PWM_SWEEP_GROUND.csv";
out_grouped = "grouped_2OUTDOOR_FIXED_PWM_SWEEP_GROUND.csv";

writetable(summary, out_summary);
writetable(grouped, out_grouped);

fprintf("\nSaved:\n");
fprintf("  %s\n", out_summary);
fprintf("  %s\n", out_grouped);

%% Plots: overview
figure("Name","PWM Sweep Overview");
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

%% Plots: IMU response
figure("Name","IMU Response vs PWM");
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
    plot(t, acc_d_res);
    grid on;
    ylabel("m/s^2");
    xlabel("time [s]");
    title("acc\_ned\_d and residual");
    legend("acc\_ned\_d", "residual");
end

%% Plots: EKF vertical
figure("Name","EKF Vertical vs PWM");
tiledlayout(4,1);

nexttile;
plot(t, pwm_mean);
grid on;
ylabel("PWM");
title("ekf\_pwm\_mean");

nexttile;
if ismember("ekf_pos_d", T.Properties.VariableNames)
    plot(t, T.ekf_pos_d);
    grid on;
    ylabel("m");
    title("ekf\_pos\_d");
end

nexttile;
if ismember("ekf_vel_d", T.Properties.VariableNames)
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

%% Plots: grouped metrics
figure("Name","Grouped Metrics by PWM");

if height(grouped) >= 2
    tiledlayout(3,1);

    nexttile;
    plot(grouped.pwm, grouped.baro_res_std_m_mean, "-o");
    grid on;
    xlabel("PWM");
    ylabel("m");
    title("Barometer residual std by PWM");

    nexttile;
    if ismember("acc_ned_d_res_std_mean", grouped.Properties.VariableNames)
        plot(grouped.pwm, grouped.acc_ned_d_res_std_mean, "-o");
        grid on;
        xlabel("PWM");
        ylabel("m/s^2");
        title("acc\_ned\_d residual std by PWM");
    end

    nexttile;
    if ismember("acc_ned_h_rms_mean", grouped.Properties.VariableNames)
        plot(grouped.pwm, grouped.acc_ned_h_rms_mean, "-o");
        grid on;
        xlabel("PWM");
        ylabel("m/s^2");
        title("acc\_ned\_h RMS by PWM");
    end
end

%% Simple interpretation guide
fprintf("\n[How to read this log]\n");
fprintf("1) baro_res_var_m2_mean increases with PWM -> R_baro PWM scale 근거.\n");
fprintf("2) acc_ned_d_res_var_mean increases with PWM -> Q_acc_z PWM scale 근거.\n");
fprintf("3) acc_ned_h_rms/P95 increases with PWM -> vibration/rotor wash disturbance 근거.\n");
fprintf("4) 1400 PWM segment is short, so use only as warning/event, not stable statistics.\n");
fprintf("5) ekf_innov_pos_d is GNSS D innovation, not baro innovation.\n");

fprintf("\nDone.\n");