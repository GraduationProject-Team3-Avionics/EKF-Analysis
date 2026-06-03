clear; clc; close all;

%% ============================================================
% STRIX FC CSV IMU/PWM Segment Analyzer
% =============================================================
%
% 목적:
%   현재 Data*.CSV / TEST*.CSV 기본 로그만으로
%   PWM 구간별 IMU 가속도계 품질을 분석한다.
%
% 분석 내용:
%   1) PWM 구간 자동 분리
%   2) 각 PWM 구간별 raw accel / ekf_accel_body / acc_ned 통계
%   3) 각 PWM 구간별 FFT
%   4) LPF 전후 감쇠 효과 추정
%   5) acc_ned_H와 EKF speed_H 관계 확인
%
% 주의:
%   SD 로그 샘플링이 20~25 Hz 수준이면 모터 고주파 진동은 정확히 못 본다.
%   이 스크립트의 FFT는 "현재 CSV에 남아있는 저주파/aliasing 성분" 분석용이다.
%   notch filter 설계용 진동 주파수는 고속 raw IMU 로그가 필요하다.

%% =========================
% 0. User Settings
% =========================

csv_file = "data\TEST_10.CSV";

% 분석 시작/끝 시간 [s]
analysis_start_sec = 0.0;
analysis_end_sec   = inf;

% PWM 구간 자동 분리 설정
pwm_round_step = 50;         % PWM을 50 단위로 반올림해서 구간화
min_segment_sec = 2.0;       % 너무 짧은 구간 제거
min_segment_samples = 20;    % 너무 적은 샘플 제거

% FFT 설정
remove_mean_before_fft = true;
use_hann_window = true;

% 보고 싶은 대표 PWM 구간 수
% 너무 많으면 figure가 많아지므로 상위 구간만 그림
max_segments_to_plot = 6;

% 관심 threshold
raw_acc_norm_threshold_g = 2.0;
acc_ned_h_threshold = 3.0;
ekf_speed_h_threshold = 0.5;

g0 = 9.80665;

%% =========================
% 1. Load CSV
% =========================

if ~isfile(csv_file)
    error("CSV 파일을 찾을 수 없습니다: %s", csv_file);
end

T = readtable(csv_file, "VariableNamingRule", "preserve");
cols = string(T.Properties.VariableNames);
N = height(T);

need(cols, "timestamp_ms");
need(cols, ["ax","ay","az"]);
need(cols, ["ekf_accel_body_x","ekf_accel_body_y","ekf_accel_body_z"]);
need(cols, ["acc_ned_n","acc_ned_e","acc_ned_d"]);
need(cols, ["ekf_vel_n","ekf_vel_e","ekf_vel_d"]);

t = double(T.timestamp_ms) * 1e-3;
t = t - t(1);

mask_time = t >= analysis_start_sec & t <= analysis_end_sec;

fprintf("\n=================================================\n");
fprintf("STRIX FC CSV IMU/PWM Segment Analyzer\n");
fprintf("=================================================\n");
fprintf("Loaded CSV : %s\n", csv_file);
fprintf("Rows       : %d\n", N);
fprintf("Columns    : %d\n", width(T));

%% =========================
% 2. Load Signals
% =========================

% PWM
if ismember("ekf_pwm_mean", cols)
    pwm = double(T.ekf_pwm_mean);
    pwm_source = "ekf_pwm_mean";
elseif all(ismember(["M1","M2","M3","M4"], cols))
    pwm = mean([double(T.M1), double(T.M2), double(T.M3), double(T.M4)], 2, "omitnan");
    pwm_source = "mean(M1:M4)";
else
    error("PWM 컬럼을 찾지 못했습니다. ekf_pwm_mean 또는 M1~M4가 필요합니다.");
end

% raw accel [g] and [m/s^2]
raw_acc_g = [double(T.ax), double(T.ay), double(T.az)];
raw_acc_mps2 = raw_acc_g * g0;
raw_acc_norm_g = sqrt(sum(raw_acc_g.^2, 2));
raw_acc_norm_mps2 = raw_acc_norm_g * g0;

% body accel after LPF and bias removal [m/s^2]
acc_body = [
    double(T.ekf_accel_body_x), ...
    double(T.ekf_accel_body_y), ...
    double(T.ekf_accel_body_z)
];
acc_body_norm = sqrt(sum(acc_body.^2, 2));

% actual propagation input [m/s^2]
acc_ned = [
    double(T.acc_ned_n), ...
    double(T.acc_ned_e), ...
    double(T.acc_ned_d)
];
acc_ned_h = hypot(acc_ned(:,1), acc_ned(:,2));
acc_ned_norm = sqrt(sum(acc_ned.^2, 2));

% velocity
ekf_vel = [
    double(T.ekf_vel_n), ...
    double(T.ekf_vel_e), ...
    double(T.ekf_vel_d)
];
ekf_speed_h = hypot(ekf_vel(:,1), ekf_vel(:,2));

if all(ismember(["gnss_vel_n_mps","gnss_vel_e_mps","gnss_vel_d_mps"], cols))
    gnss_vel = [
        double(T.gnss_vel_n_mps), ...
        double(T.gnss_vel_e_mps), ...
        double(T.gnss_vel_d_mps)
    ];
    gnss_speed_h = hypot(gnss_vel(:,1), gnss_vel(:,2));
else
    gnss_speed_h = nan(N,1);
end

roll_deg  = col(T, cols, "roll_deg", col(T, cols, "ekf_roll_deg", nan(N,1)));
pitch_deg = col(T, cols, "pitch_deg", col(T, cols, "ekf_pitch_deg", nan(N,1)));

%% =========================
% 3. Estimate Sampling Rate
% =========================

valid_t = isfinite(t);
dt = diff(t(valid_t));
dt = dt(dt > 0);

fs_mean = 1 / mean(dt);
fs_median = 1 / median(dt);
nyq = fs_median / 2;

fprintf("PWM source : %s\n", pwm_source);
fprintf("Mean fs    : %.3f Hz\n", fs_mean);
fprintf("Median fs  : %.3f Hz\n", fs_median);
fprintf("Nyquist    : %.3f Hz\n", nyq);

%% =========================
% 4. PWM Segment Detection
% =========================

pwm_round = round(pwm / pwm_round_step) * pwm_round_step;
pwm_round(~isfinite(pwm_round)) = NaN;

valid_seg_base = mask_time & isfinite(pwm_round) & isfinite(t);

segments = struct([]);
seg_count = 0;

i = 1;
while i <= N
    if ~valid_seg_base(i)
        i = i + 1;
        continue;
    end

    current_pwm = pwm_round(i);
    j = i;

    while j <= N && valid_seg_base(j) && pwm_round(j) == current_pwm
        j = j + 1;
    end

    idx = i:(j-1);
    duration = t(idx(end)) - t(idx(1));

    if duration >= min_segment_sec && numel(idx) >= min_segment_samples
        seg_count = seg_count + 1;
        segments(seg_count).idx = idx;
        segments(seg_count).pwm_label = current_pwm;
        segments(seg_count).pwm_mean = meanf(pwm(idx));
        segments(seg_count).t_start = t(idx(1));
        segments(seg_count).t_end = t(idx(end));
        segments(seg_count).duration = duration;
        segments(seg_count).samples = numel(idx);
    end

    i = j;
end

if isempty(segments)
    error("유효한 PWM 구간을 찾지 못했습니다. min_segment_sec 또는 pwm_round_step을 조정하세요.");
end

fprintf("\nDetected PWM segments: %d\n", numel(segments));

%% =========================
% 5. Segment Statistics
% =========================

summary = table();

for k = 1:numel(segments)
    idx = segments(k).idx;

    row = table();
    row.seg_id = k;
    row.pwm_label = segments(k).pwm_label;
    row.pwm_mean = segments(k).pwm_mean;
    row.t_start = segments(k).t_start;
    row.t_end = segments(k).t_end;
    row.duration = segments(k).duration;
    row.samples = segments(k).samples;

    row.raw_norm_mean_g = meanf(raw_acc_norm_g(idx));
    row.raw_norm_std_g = stdf(raw_acc_norm_g(idx));
    row.raw_norm_rms_g = rmsf(raw_acc_norm_g(idx) - meanf(raw_acc_norm_g(idx)));
    row.raw_norm_max_g = maxf(raw_acc_norm_g(idx));

    row.body_norm_mean = meanf(acc_body_norm(idx));
    row.body_norm_std = stdf(acc_body_norm(idx));
    row.body_norm_rms = rmsf(acc_body_norm(idx) - meanf(acc_body_norm(idx)));
    row.body_norm_max = maxf(acc_body_norm(idx));

    row.acc_ned_h_mean = meanf(acc_ned_h(idx));
    row.acc_ned_h_std = stdf(acc_ned_h(idx));
    row.acc_ned_h_rms = rmsf(acc_ned_h(idx));
    row.acc_ned_h_max = maxf(acc_ned_h(idx));

    row.ekf_speed_h_mean = meanf(ekf_speed_h(idx));
    row.ekf_speed_h_max = maxf(ekf_speed_h(idx));

    row.gnss_speed_h_mean = meanf(gnss_speed_h(idx));
    row.gnss_speed_h_max = maxf(gnss_speed_h(idx));

    row.roll_mean_deg = meanf(roll_deg(idx));
    row.roll_std_deg = stdf(roll_deg(idx));
    row.pitch_mean_deg = meanf(pitch_deg(idx));
    row.pitch_std_deg = stdf(pitch_deg(idx));

    row.raw_over_2g_pct = 100 * mean(raw_acc_norm_g(idx) > raw_acc_norm_threshold_g, "omitnan");
    row.acc_ned_h_over_th_pct = 100 * mean(acc_ned_h(idx) > acc_ned_h_threshold, "omitnan");
    row.speed_h_over_th_pct = 100 * mean(ekf_speed_h(idx) > ekf_speed_h_threshold, "omitnan");

    % FFT dominant frequency per segment
    [f_ax, P_ax] = simple_fft(t(idx), raw_acc_mps2(idx,1), remove_mean_before_fft, use_hann_window);
    [f_ay, P_ay] = simple_fft(t(idx), raw_acc_mps2(idx,2), remove_mean_before_fft, use_hann_window);
    [f_az, P_az] = simple_fft(t(idx), raw_acc_mps2(idx,3), remove_mean_before_fft, use_hann_window);
    [f_h, P_h] = simple_fft(t(idx), acc_ned_h(idx), remove_mean_before_fft, use_hann_window);

    row.dom_ax_hz = dominant_freq(f_ax, P_ax);
    row.dom_ay_hz = dominant_freq(f_ay, P_ay);
    row.dom_az_hz = dominant_freq(f_az, P_az);
    row.dom_acc_ned_h_hz = dominant_freq(f_h, P_h);

    % LPF reduction approximation:
    % compare raw horizontal-like xy magnitude vs acc_body xy magnitude.
    raw_xy = hypot(raw_acc_mps2(idx,1), raw_acc_mps2(idx,2));
    body_xy = hypot(acc_body(idx,1), acc_body(idx,2));
    row.xy_rms_raw = rmsf(raw_xy - meanf(raw_xy));
    row.xy_rms_body = rmsf(body_xy - meanf(body_xy));
    % 주의: raw ax/ay와 body x/y는 IMU->body 변환 및 중력 성분 영향 때문에
    % 완전히 같은 물리량은 아니다. 이 값은 "대략적인 변동 감소 참고용"으로만 본다.
    row.xy_lpf_reduction_pct = 100 * (1 - row.xy_rms_body / max(row.xy_rms_raw, eps));

    summary = [summary; row]; %#ok<AGROW>
end

fprintf("\n=================================================\n");
fprintf("PWM Segment Summary\n");
fprintf("=================================================\n");
disp(summary);

writetable(summary, "strix_pwm_segment_summary.csv");
fprintf("Saved summary CSV: strix_pwm_segment_summary.csv\n");

%% =========================
% 6. Select Segments To Plot
% =========================

% 우선순위: PWM 평균이 높은 순 + duration 충분한 순
[~, order] = sortrows([summary.pwm_mean, summary.duration], [-1 -2]);
plot_seg_ids = summary.seg_id(order(1:min(max_segments_to_plot, height(summary))));

%% =========================
% 7. Figure 1: Time Overview
% =========================

figure("Name", "01 Time Overview");
set(gcf, "Color", "w");

subplot(6,1,1);
hold on; grid on;
plot(t, pwm, "LineWidth", 1.1);
ylabel("PWM");
title("PWM");

subplot(6,1,2);
hold on; grid on;
plot(t, raw_acc_norm_g, "LineWidth", 1.0);
yline(1.0, "--");
yline(raw_acc_norm_threshold_g, ":");
ylabel("|raw acc| [g]");
title("Raw accel norm");

subplot(6,1,3);
hold on; grid on;
plot(t, acc_body_norm, "LineWidth", 1.0);
ylabel("[m/s²]");
title("ekf_accel_body norm after LPF and bias removal");

subplot(6,1,4);
hold on; grid on;
plot(t, acc_ned_h, "LineWidth", 1.0);
yline(acc_ned_h_threshold, ":");
ylabel("[m/s²]");
title("acc_ned horizontal");

subplot(6,1,5);
hold on; grid on;
plot(t, ekf_speed_h, "LineWidth", 1.0);
plot(t, gnss_speed_h, "--", "LineWidth", 1.0);
yline(ekf_speed_h_threshold, ":");
ylabel("[m/s]");
title("Horizontal speed");
legend("EKF", "GNSS", "threshold", "Location", "best");

subplot(6,1,6);
hold on; grid on;
plot(t, roll_deg, "LineWidth", 1.0);
plot(t, pitch_deg, "LineWidth", 1.0);
ylabel("[deg]");
xlabel("Time [s]");
title("Roll / Pitch");
legend("roll", "pitch", "Location", "best");

%% =========================
% 8. Figure 2: Segment Bar Summary
% =========================

figure("Name", "02 PWM Segment Bar Summary");
set(gcf, "Color", "w");

% PWM 1000, 1400처럼 같은 PWM 구간이 여러 번 나올 수 있으므로
% categorical label은 segment id를 붙여서 반드시 unique하게 만든다.
x_labels = strings(height(summary), 1);
for ii = 1:height(summary)
    x_labels(ii) = sprintf("S%d_%d", summary.seg_id(ii), round(summary.pwm_mean(ii)));
end
x = categorical(x_labels);
x = reordercats(x, x_labels);

subplot(4,1,1);
bar(x, summary.raw_norm_rms_g);
grid on;
ylabel("RMS [g]");
title("Raw accel norm fluctuation RMS by PWM segment");

subplot(4,1,2);
bar(x, summary.acc_ned_h_rms);
grid on;
ylabel("RMS [m/s²]");
title("acc_ned horizontal RMS by PWM segment");

subplot(4,1,3);
bar(x, summary.ekf_speed_h_max);
grid on;
ylabel("max [m/s]");
title("EKF horizontal speed max by PWM segment");

subplot(4,1,4);
bar(x, summary.raw_over_2g_pct);
grid on;
ylabel("[%]");
xlabel("PWM segment");
title("Percentage of raw accel norm > threshold");

%% =========================
% 9. Figure 3: Raw Accel FFT by PWM Segment
% =========================

figure("Name", "03 Raw Accel FFT by PWM Segment");
set(gcf, "Color", "w");

nplot = numel(plot_seg_ids);
for p = 1:nplot
    k = plot_seg_ids(p);
    idx = segments(k).idx;

    [f_ax, P_ax] = simple_fft(t(idx), raw_acc_mps2(idx,1), remove_mean_before_fft, use_hann_window);
    [f_ay, P_ay] = simple_fft(t(idx), raw_acc_mps2(idx,2), remove_mean_before_fft, use_hann_window);
    [f_az, P_az] = simple_fft(t(idx), raw_acc_mps2(idx,3), remove_mean_before_fft, use_hann_window);

    subplot(nplot,1,p);
    hold on; grid on;
    plot(f_ax, P_ax, "LineWidth", 1.0);
    plot(f_ay, P_ay, "LineWidth", 1.0);
    plot(f_az, P_az, "LineWidth", 1.0);
    xlim([0, nyq]);
    ylabel("Amp");
    title(sprintf("Raw accel FFT - PWM %.0f, %.1f~%.1fs", ...
        segments(k).pwm_mean, segments(k).t_start, segments(k).t_end));
    legend("ax", "ay", "az", "Location", "best");
end
xlabel("Frequency [Hz]");

%% =========================
% 10. Figure 4: acc_ned_H FFT by PWM Segment
% =========================

figure("Name", "04 acc_ned_H FFT by PWM Segment");
set(gcf, "Color", "w");

for p = 1:nplot
    k = plot_seg_ids(p);
    idx = segments(k).idx;

    [f_h, P_h] = simple_fft(t(idx), acc_ned_h(idx), remove_mean_before_fft, use_hann_window);
    [f_speed, P_speed] = simple_fft(t(idx), ekf_speed_h(idx), remove_mean_before_fft, use_hann_window);

    subplot(nplot,1,p);
    hold on; grid on;
    plot(f_h, P_h, "LineWidth", 1.0);
    plot(f_speed, P_speed, "LineWidth", 1.0);
    xlim([0, nyq]);
    ylabel("Amp");
    title(sprintf("acc_ned_H / EKF speed FFT - PWM %.0f, %.1f~%.1fs", ...
        segments(k).pwm_mean, segments(k).t_start, segments(k).t_end));
    legend("acc_ned_H", "EKF speed_H", "Location", "best");
end
xlabel("Frequency [Hz]");

%% =========================
% 11. Figure 5: Per Segment Time Windows
% =========================

for p = 1:nplot
    k = plot_seg_ids(p);
    idx = segments(k).idx;

    figure("Name", sprintf("05 Segment Time PWM %.0f", segments(k).pwm_mean));
    set(gcf, "Color", "w");

    subplot(5,1,1);
    hold on; grid on;
    plot(t(idx), pwm(idx), "LineWidth", 1.1);
    ylabel("PWM");
    title(sprintf("PWM Segment %.0f, %.1f~%.1fs", ...
        segments(k).pwm_mean, segments(k).t_start, segments(k).t_end));

    subplot(5,1,2);
    hold on; grid on;
    plot(t(idx), raw_acc_g(idx,1), "LineWidth", 1.0);
    plot(t(idx), raw_acc_g(idx,2), "LineWidth", 1.0);
    plot(t(idx), raw_acc_g(idx,3), "LineWidth", 1.0);
    plot(t(idx), raw_acc_norm_g(idx), "k", "LineWidth", 1.0);
    yline(1.0, "--");
    ylabel("[g]");
    title("Raw accel");
    legend("ax", "ay", "az", "|a|", "Location", "best");

    subplot(5,1,3);
    hold on; grid on;
    plot(t(idx), acc_body(idx,1), "LineWidth", 1.0);
    plot(t(idx), acc_body(idx,2), "LineWidth", 1.0);
    plot(t(idx), acc_body(idx,3), "LineWidth", 1.0);
    ylabel("[m/s²]");
    title("ekf_accel_body after LPF/bias");
    legend("body x", "body y", "body z", "Location", "best");

    subplot(5,1,4);
    hold on; grid on;
    plot(t(idx), acc_ned(idx,1), "LineWidth", 1.0);
    plot(t(idx), acc_ned(idx,2), "LineWidth", 1.0);
    plot(t(idx), acc_ned_h(idx), "LineWidth", 1.0);
    yline(acc_ned_h_threshold, ":");
    ylabel("[m/s²]");
    title("acc_ned N/E/H");
    legend("N", "E", "H", "Location", "best");

    subplot(5,1,5);
    hold on; grid on;
    plot(t(idx), ekf_speed_h(idx), "LineWidth", 1.0);
    plot(t(idx), gnss_speed_h(idx), "--", "LineWidth", 1.0);
    yline(ekf_speed_h_threshold, ":");
    ylabel("[m/s]");
    xlabel("Time [s]");
    title("Horizontal speed");
    legend("EKF", "GNSS", "threshold", "Location", "best");
end

%% =========================
% 12. Save MAT Report
% =========================

report = struct();
report.csv_file = csv_file;
report.t = t;
report.pwm = pwm;
report.raw_acc_g = raw_acc_g;
report.raw_acc_norm_g = raw_acc_norm_g;
report.acc_body = acc_body;
report.acc_body_norm = acc_body_norm;
report.acc_ned = acc_ned;
report.acc_ned_h = acc_ned_h;
report.ekf_speed_h = ekf_speed_h;
report.gnss_speed_h = gnss_speed_h;
report.roll_deg = roll_deg;
report.pitch_deg = pitch_deg;
report.segments = segments;
report.summary = summary;
report.fs_mean = fs_mean;
report.fs_median = fs_median;
report.nyquist = nyq;

% save("strix_pwm_imu_segment_analysis_report.mat", "report");
% fprintf("Saved MAT report: strix_pwm_imu_segment_analysis_report.mat\n");

%% ============================================================
% Local Functions
% ============================================================

function need(cols, names)
names = string(names);
for k = 1:numel(names)
    if ~ismember(names(k), cols)
        error("CSV에 필요한 컬럼이 없습니다: %s", names(k));
    end
end
end

function x = col(T, cols, name, default_value)
if ismember(name, cols)
    x = double(T.(name));
else
    x = default_value;
end
end

function y = meanf(x)
x = x(:); x = x(isfinite(x));
if isempty(x), y = NaN; else, y = mean(x); end
end

function y = stdf(x)
x = x(:); x = x(isfinite(x));
if isempty(x), y = NaN; else, y = std(x); end
end

function y = rmsf(x)
x = x(:); x = x(isfinite(x));
if isempty(x), y = NaN; else, y = sqrt(mean(x.^2)); end
end

function y = maxf(x)
x = x(:); x = x(isfinite(x));
if isempty(x), y = NaN; else, y = max(x); end
end

function dxdt = grad_safe(x, t)
x = x(:); t = t(:);
dxdt = nan(size(x));
valid = isfinite(x) & isfinite(t);
if sum(valid) < 3
    return;
end
xv = x(valid);
tv = t(valid);
[tu, ia] = unique(tv, "stable");
xu = xv(ia);
if numel(tu) < 3
    return;
end
du = gradient(xu, tu);
dxdt(valid) = interp1(tu, du, tv, "linear", "extrap");
end

function [f, P1] = simple_fft(t, x, remove_mean, use_hann)
t = t(:);
x = x(:);

valid = isfinite(t) & isfinite(x);
t = t(valid);
x = x(valid);

f = [];
P1 = [];

if numel(x) < 8
    return;
end

% uniform interpolation
[t_unique, ia] = unique(t, "stable");
x_unique = x(ia);

if numel(t_unique) < 8
    return;
end

dt = median(diff(t_unique));
if ~isfinite(dt) || dt <= 0
    return;
end

fs = 1 / dt;
t_uniform = (t_unique(1):dt:t_unique(end)).';
x_uniform = interp1(t_unique, x_unique, t_uniform, "linear", "extrap");

if remove_mean
    x_uniform = x_uniform - mean(x_uniform, "omitnan");
end

L = numel(x_uniform);

if use_hann
    w = hann(L);
    x_uniform = x_uniform .* w;
    amp_corr = mean(w);
else
    amp_corr = 1;
end

Y = fft(x_uniform);
P2 = abs(Y / L) / max(amp_corr, eps);
P1 = P2(1:floor(L/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);
f = fs * (0:floor(L/2)) / L;
end

function fd = dominant_freq(f, P)
% dominant frequency helper
% f와 P가 row/column 방향이 달라도 논리 인덱스 오류가 나지 않도록
% 둘 다 column vector로 강제 변환한다.

f = f(:);
P = P(:);

if isempty(f) || isempty(P) || numel(f) < 2 || numel(P) < 2
    fd = NaN;
    return;
end

L = min(numel(f), numel(P));
f = f(1:L);
P = P(1:L);

% DC 제외, 너무 낮은 0 근처 제외
valid = (f > 0.1) & isfinite(f) & isfinite(P);

if ~any(valid)
    fd = NaN;
    return;
end

fv = f(valid);
Pv = P(valid);

[~, i] = max(Pv);
fd = fv(i);
end
