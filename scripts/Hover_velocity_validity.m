clear; clc; close all;

%% ============================================================
%  EKF XY Hover Readiness Analysis
%  - 목적:
%    RTK Ground Truth 없이, 현재 EKF 상태값으로
%    XY 제자리 호버링 제어를 걸어도 될지 사전 판단
%
%  핵심:
%    1) EKF XY velocity 안정성
%    2) EKF XY position scatter
%    3) GNSS update에 의한 state jump
%    4) GNSS hAcc와 EKF 흔들림 관계
%
%  D축은 barometer로 별도 평가 예정이므로 제외
% ============================================================

% filename = "data/TEST_10.CSV";
filename = "260610/fc_damp_03.CSV";

% 분석 구간 설정
% 전체를 쓰려면 use_manual_time_range = false
use_manual_time_range = false;
analysis_start_sec = 0.0;
analysis_end_sec   = 9999.0;

% GNSS update가 수행된 샘플만 따로 보고 싶은 경우
% Hover readiness에서는 전체 state 흔들림이 중요하므로 기본 false 추천
use_only_gnss_update = false;

% 정지 상태 판정 기준
% 기체가 실제로 고정/정지된 로그라면 이 기준으로 평가
good_vel_rms_threshold = 0.05;    % [m/s] 매우 좋음
ok_vel_rms_threshold   = 0.15;    % [m/s] 어느 정도 가능
bad_vel_rms_threshold  = 0.30;    % [m/s] 위험

good_pos_std_threshold = 0.20;    % [m] 정지 위치 흔들림 좋음
ok_pos_std_threshold   = 0.50;    % [m] 어느 정도 가능
bad_pos_std_threshold  = 1.00;    % [m] 위치제어 흔들림 가능성 큼

%% ============================================================
%  1. CSV Load
% ============================================================
T = readtable(filename, "VariableNamingRule", "preserve");
vars = string(T.Properties.VariableNames);

t = T.timestamp_ms / 1000;
t = t - t(1);

%% ============================================================
%  2. Basic Valid Mask
% ============================================================
mask = true(height(T), 1);

if ismember("ekf_ready", vars)
    mask = mask & (T.ekf_ready == 1);
end

if ismember("gnss_ref_ready", vars)
    mask = mask & (T.gnss_ref_ready == 1);
end

if ismember("gnss_valid", vars)
    mask = mask & (T.gnss_valid == 1);
end

if use_only_gnss_update && ismember("gnss_update_executed", vars)
    mask = mask & (T.gnss_update_executed == 1);
end

if use_manual_time_range
    mask = mask & (t >= analysis_start_sec) & (t <= analysis_end_sec);
end

%% ============================================================
%  3. Required Column Check
% ============================================================
required_cols = [
    "ekf_pos_n"
    "ekf_pos_e"
    "ekf_vel_n"
    "ekf_vel_e"
];

for i = 1:numel(required_cols)
    if ~ismember(required_cols(i), vars)
        error("Required column missing: %s", required_cols(i));
    end
end

%% ============================================================
%  4. Extract EKF XY Position / Velocity
% ============================================================
pos_n = T.ekf_pos_n;
pos_e = T.ekf_pos_e;

vel_n = T.ekf_vel_n;
vel_e = T.ekf_vel_e;

% XY velocity magnitude
vel_xy = sqrt(vel_n.^2 + vel_e.^2);

% 정지 위치 기준: 평가 구간 평균 위치를 기준점으로 둠
pos_n_mean = mean(pos_n(mask), "omitnan");
pos_e_mean = mean(pos_e(mask), "omitnan");

pos_n_centered = pos_n - pos_n_mean;
pos_e_centered = pos_e - pos_e_mean;

% 평균 위치로부터의 수평 반경
pos_radius = sqrt(pos_n_centered.^2 + pos_e_centered.^2);

%% ============================================================
%  5. Optional GNSS Data
% ============================================================
has_gnss_pos = ismember("ekf_gnss_pos_n", vars) && ismember("ekf_gnss_pos_e", vars);

if has_gnss_pos
    gnss_n = T.ekf_gnss_pos_n;
    gnss_e = T.ekf_gnss_pos_e;

    gnss_ekf_err_n = gnss_n - pos_n;
    gnss_ekf_err_e = gnss_e - pos_e;
    gnss_ekf_herr = sqrt(gnss_ekf_err_n.^2 + gnss_ekf_err_e.^2);
else
    gnss_n = nan(height(T), 1);
    gnss_e = nan(height(T), 1);
    gnss_ekf_herr = nan(height(T), 1);
end

if ismember("gnss_hacc_m", vars)
    hacc = T.gnss_hacc_m;
else
    hacc = nan(height(T), 1);
end

if ismember("gnss_sacc_mps", vars)
    sacc = T.gnss_sacc_mps;
else
    sacc = nan(height(T), 1);
end

%% ============================================================
%  6. Optional GNSS Update Delta
% ============================================================
has_delta_vel = ismember("delta_vel_gnss_update_n", vars) && ...
                ismember("delta_vel_gnss_update_e", vars);

if has_delta_vel
    delta_vel_n = T.delta_vel_gnss_update_n;
    delta_vel_e = T.delta_vel_gnss_update_e;
    delta_vel_xy = sqrt(delta_vel_n.^2 + delta_vel_e.^2);
else
    delta_vel_n = nan(height(T), 1);
    delta_vel_e = nan(height(T), 1);
    delta_vel_xy = nan(height(T), 1);
end

has_delta_pos = ismember("delta_pos_gnss_update_n", vars) && ...
                ismember("delta_pos_gnss_update_e", vars);

if has_delta_pos
    delta_pos_n = T.delta_pos_gnss_update_n;
    delta_pos_e = T.delta_pos_gnss_update_e;
    delta_pos_xy = sqrt(delta_pos_n.^2 + delta_pos_e.^2);
else
    delta_pos_xy = nan(height(T), 1);
end

if ismember("gnss_update_executed", vars)
    gnss_update = T.gnss_update_executed == 1;
else
    gnss_update = false(height(T), 1);
end

%% ============================================================
%  7. Optional PWM / ARM Information
% ============================================================
motor_cols = ["motor1_pwm", "motor2_pwm", "motor3_pwm", "motor4_pwm", ...
              "pwm1", "pwm2", "pwm3", "pwm4", ...
              "M1", "M2", "M3", "M4"];

available_motor_cols = motor_cols(ismember(motor_cols, vars));

has_pwm = numel(available_motor_cols) >= 4;

if has_pwm
    pwm_mat = zeros(height(T), numel(available_motor_cols));
    for i = 1:numel(available_motor_cols)
        pwm_mat(:, i) = T.(available_motor_cols(i));
    end
    pwm_mean = mean(pwm_mat, 2, "omitnan");
else
    pwm_mean = nan(height(T), 1);
end

%% ============================================================
%  8. Statistics
% ============================================================
valid_t = t(mask);

pos_radius_valid = pos_radius(mask);
vel_xy_valid = vel_xy(mask);

pos_n_valid = pos_n_centered(mask);
pos_e_valid = pos_e_centered(mask);

vel_n_valid = vel_n(mask);
vel_e_valid = vel_e(mask);

% Position stability
pos_radius_rms = rms(pos_radius_valid);
pos_radius_mean = mean(pos_radius_valid, "omitnan");
pos_radius_std = std(pos_radius_valid, "omitnan");
pos_radius_p95 = prctile(pos_radius_valid, 95);
pos_radius_max = max(pos_radius_valid);

pos_n_std = std(pos_n_valid, "omitnan");
pos_e_std = std(pos_e_valid, "omitnan");

% Velocity stability
vel_xy_rms = rms(vel_xy_valid);
vel_xy_mean = mean(vel_xy_valid, "omitnan");
vel_xy_std = std(vel_xy_valid, "omitnan");
vel_xy_p95 = prctile(vel_xy_valid, 95);
vel_xy_max = max(vel_xy_valid);

vel_n_mean = mean(vel_n_valid, "omitnan");
vel_e_mean = mean(vel_e_valid, "omitnan");
vel_n_std = std(vel_n_valid, "omitnan");
vel_e_std = std(vel_e_valid, "omitnan");

% Velocity threshold coverage
vel_under_005 = mean(vel_xy_valid < 0.05, "omitnan") * 100;
vel_under_010 = mean(vel_xy_valid < 0.10, "omitnan") * 100;
vel_under_015 = mean(vel_xy_valid < 0.15, "omitnan") * 100;
vel_under_030 = mean(vel_xy_valid < 0.30, "omitnan") * 100;

% Position radius coverage
pos_under_020 = mean(pos_radius_valid < 0.20, "omitnan") * 100;
pos_under_050 = mean(pos_radius_valid < 0.50, "omitnan") * 100;
pos_under_100 = mean(pos_radius_valid < 1.00, "omitnan") * 100;

% GNSS update delta statistics
if has_delta_vel
    delta_vel_xy_valid = delta_vel_xy(mask & gnss_update);
    delta_vel_xy_rms = rms(delta_vel_xy_valid);
    delta_vel_xy_p95 = prctile(delta_vel_xy_valid, 95);
    delta_vel_xy_max = max(delta_vel_xy_valid);
else
    delta_vel_xy_rms = nan;
    delta_vel_xy_p95 = nan;
    delta_vel_xy_max = nan;
end

if has_delta_pos
    delta_pos_xy_valid = delta_pos_xy(mask & gnss_update);
    delta_pos_xy_rms = rms(delta_pos_xy_valid);
    delta_pos_xy_p95 = prctile(delta_pos_xy_valid, 95);
    delta_pos_xy_max = max(delta_pos_xy_valid);
else
    delta_pos_xy_rms = nan;
    delta_pos_xy_p95 = nan;
    delta_pos_xy_max = nan;
end

%% ============================================================
%  9. Console Summary
% ============================================================
fprintf("\n=================================================\n");
fprintf("EKF XY Hover Readiness Analysis\n");
fprintf("=================================================\n");
fprintf("File              : %s\n", filename);
fprintf("Total rows         : %d\n", height(T));
fprintf("Valid samples      : %d\n", sum(mask));
fprintf("Time range         : %.3f ~ %.3f sec\n", min(valid_t), max(valid_t));
fprintf("-------------------------------------------------\n");

fprintf("[1] EKF XY Velocity Stability\n");
fprintf("vel_xy RMS         : %.4f m/s\n", vel_xy_rms);
fprintf("vel_xy mean        : %.4f m/s\n", vel_xy_mean);
fprintf("vel_xy std         : %.4f m/s\n", vel_xy_std);
fprintf("vel_xy 95%%         : %.4f m/s\n", vel_xy_p95);
fprintf("vel_xy max         : %.4f m/s\n", vel_xy_max);
fprintf("vel_n mean/std     : %.4f / %.4f m/s\n", vel_n_mean, vel_n_std);
fprintf("vel_e mean/std     : %.4f / %.4f m/s\n", vel_e_mean, vel_e_std);
fprintf("P(vel_xy < 0.05)   : %.2f %%\n", vel_under_005);
fprintf("P(vel_xy < 0.10)   : %.2f %%\n", vel_under_010);
fprintf("P(vel_xy < 0.15)   : %.2f %%\n", vel_under_015);
fprintf("P(vel_xy < 0.30)   : %.2f %%\n", vel_under_030);

fprintf("-------------------------------------------------\n");
fprintf("[2] EKF XY Position Scatter Around Mean Position\n");
fprintf("pos_radius RMS     : %.4f m\n", pos_radius_rms);
fprintf("pos_radius mean    : %.4f m\n", pos_radius_mean);
fprintf("pos_radius std     : %.4f m\n", pos_radius_std);
fprintf("pos_radius 95%%      : %.4f m\n", pos_radius_p95);
fprintf("pos_radius max     : %.4f m\n", pos_radius_max);
fprintf("N pos std          : %.4f m\n", pos_n_std);
fprintf("E pos std          : %.4f m\n", pos_e_std);
fprintf("P(radius < 0.20 m) : %.2f %%\n", pos_under_020);
fprintf("P(radius < 0.50 m) : %.2f %%\n", pos_under_050);
fprintf("P(radius < 1.00 m) : %.2f %%\n", pos_under_100);

fprintf("-------------------------------------------------\n");
fprintf("[3] GNSS Quality Reference\n");
if any(~isnan(hacc(mask)))
    fprintf("GNSS hAcc median   : %.4f m\n", median(hacc(mask), "omitnan"));
    fprintf("GNSS hAcc mean     : %.4f m\n", mean(hacc(mask), "omitnan"));
    fprintf("GNSS hAcc max      : %.4f m\n", max(hacc(mask)));
else
    fprintf("GNSS hAcc column not found.\n");
end

if any(~isnan(sacc(mask)))
    fprintf("GNSS sAcc median   : %.4f m/s\n", median(sacc(mask), "omitnan"));
    fprintf("GNSS sAcc mean     : %.4f m/s\n", mean(sacc(mask), "omitnan"));
    fprintf("GNSS sAcc max      : %.4f m/s\n", max(sacc(mask)));
end

fprintf("-------------------------------------------------\n");
fprintf("[4] GNSS Update Jump\n");
if has_delta_vel
    fprintf("delta_vel_xy RMS   : %.4f m/s\n", delta_vel_xy_rms);
    fprintf("delta_vel_xy 95%%    : %.4f m/s\n", delta_vel_xy_p95);
    fprintf("delta_vel_xy max   : %.4f m/s\n", delta_vel_xy_max);
else
    fprintf("delta_vel_gnss_update_n/e columns not found.\n");
end

if has_delta_pos
    fprintf("delta_pos_xy RMS   : %.4f m\n", delta_pos_xy_rms);
    fprintf("delta_pos_xy 95%%    : %.4f m\n", delta_pos_xy_p95);
    fprintf("delta_pos_xy max   : %.4f m\n", delta_pos_xy_max);
else
    fprintf("delta_pos_gnss_update_n/e columns not found.\n");
end
fprintf("=================================================\n\n");

%% ============================================================
%  10. Hover Readiness Judgment
% ============================================================
fprintf("=================================================\n");
fprintf("Hover Readiness Judgment\n");
fprintf("=================================================\n");

% Velocity judgment
if vel_xy_rms < good_vel_rms_threshold
    vel_judgment = "GOOD";
    fprintf("[GOOD] EKF XY velocity RMS is very small. Velocity estimate is suitable for initial hover tests.\n");
elseif vel_xy_rms < ok_vel_rms_threshold
    vel_judgment = "OK";
    fprintf("[OK] EKF XY velocity RMS is acceptable, but position hold gains should be conservative.\n");
elseif vel_xy_rms < bad_vel_rms_threshold
    vel_judgment = "CAUTION";
    fprintf("[CAUTION] EKF XY velocity RMS is somewhat large. Try velocity hold carefully before position hold.\n");
else
    vel_judgment = "BAD";
    fprintf("[BAD] EKF XY velocity RMS is too large. Position hold hover may become unstable.\n");
end

% Position judgment
if pos_radius_rms < good_pos_std_threshold
    pos_judgment = "GOOD";
    fprintf("[GOOD] EKF XY position scatter is small around the mean position.\n");
elseif pos_radius_rms < ok_pos_std_threshold
    pos_judgment = "OK";
    fprintf("[OK] EKF XY position scatter is moderate. Use low position P gain.\n");
elseif pos_radius_rms < bad_pos_std_threshold
    pos_judgment = "CAUTION";
    fprintf("[CAUTION] EKF XY position scatter is large. Position hold may wander.\n");
else
    pos_judgment = "BAD";
    fprintf("[BAD] EKF XY position scatter is too large for stable GNSS-based hover.\n");
end

fprintf("\nRecommended next step:\n");

if vel_judgment == "GOOD" || vel_judgment == "OK"
    fprintf("- Try XY velocity hold first with very low velocity/acceleration limits.\n");
    fprintf("- After velocity hold is stable, try XY position hold with low position gain.\n");
else
    fprintf("- Do NOT start with XY position hold.\n");
    fprintf("- First improve EKF velocity stability: IMU vibration, GNSS velocity R, update gate, delay.\n");
end

fprintf("=================================================\n");

%% ============================================================
%  11. Figure 1: EKF XY Hover Overview
% ============================================================
figure("Name", "EKF XY Hover Overview", "Color", "w");
tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

% Position radius
nexttile;
plot(t(mask), pos_radius(mask), "LineWidth", 1.6); hold on;
yline(0.20, "--", "0.20 m");
yline(0.50, ":", "0.50 m");
yline(1.00, "-.", "1.00 m");
grid on;
xlabel("Time [s]");
ylabel("Radius from mean [m]");
title("EKF XY Position Scatter Around Mean Position");
legend("XY radius", "0.20 m", "0.50 m", "1.00 m", "Location", "best");

% Velocity magnitude
nexttile;
plot(t(mask), vel_xy(mask), "LineWidth", 1.6); hold on;
yline(0.05, "--", "0.05 m/s");
yline(0.10, ":", "0.10 m/s");
yline(0.15, "-.", "0.15 m/s");
yline(0.30, "-", "0.30 m/s");
grid on;
xlabel("Time [s]");
ylabel("Velocity magnitude [m/s]");
title("EKF XY Velocity Magnitude");
legend("v_{xy}", "0.05", "0.10", "0.15", "0.30", "Location", "best");

% GNSS hAcc
nexttile;
if any(~isnan(hacc(mask)))
    plot(t(mask), hacc(mask), "LineWidth", 1.6); hold on;
end
if any(~isnan(sacc(mask)))
    plot(t(mask), sacc(mask), "LineWidth", 1.3);
end
grid on;
xlabel("Time [s]");
ylabel("GNSS accuracy");
title("GNSS hAcc / sAcc");
legend("hAcc [m]", "sAcc [m/s]", "Location", "best");

%% ============================================================
%  12. Figure 2: EKF XY Position Scatter Plot
% ============================================================
figure("Name", "EKF XY Position Scatter", "Color", "w");

plot(pos_e_centered(mask), pos_n_centered(mask), ".", "MarkerSize", 8); hold on;
plot(0, 0, "kx", "MarkerSize", 10, "LineWidth", 2);

axis equal;
grid on;
xlabel("E position from mean [m]");
ylabel("N position from mean [m]");
title("EKF XY Position Scatter Around Mean Position");

% reference circles
theta = linspace(0, 2*pi, 300);
r_list = [0.2, 0.5, 1.0];

for r = r_list
    plot(r*cos(theta), r*sin(theta), "--", "LineWidth", 1.0);
end

legend("EKF XY samples", "Mean position", "0.2 m", "0.5 m", "1.0 m", "Location", "best");

%% ============================================================
%  13. Figure 3: N/E Position and Velocity
% ============================================================
figure("Name", "EKF N/E Position and Velocity", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t(mask), pos_n_centered(mask), "LineWidth", 1.4); hold on;
yline(0, "k--");
grid on;
xlabel("Time [s]");
ylabel("N pos [m]");
title("EKF N Position From Mean");

nexttile;
plot(t(mask), pos_e_centered(mask), "LineWidth", 1.4); hold on;
yline(0, "k--");
grid on;
xlabel("Time [s]");
ylabel("E pos [m]");
title("EKF E Position From Mean");

nexttile;
plot(t(mask), vel_n(mask), "LineWidth", 1.4); hold on;
yline(0, "k--");
grid on;
xlabel("Time [s]");
ylabel("N vel [m/s]");
title("EKF N Velocity");

nexttile;
plot(t(mask), vel_e(mask), "LineWidth", 1.4); hold on;
yline(0, "k--");
grid on;
xlabel("Time [s]");
ylabel("E vel [m/s]");
title("EKF E Velocity");

%% ============================================================
%  14. Figure 4: GNSS Update Jump
% ============================================================
figure("Name", "GNSS Update Effect on EKF State", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
if has_delta_vel
    plot(t(mask), delta_vel_xy(mask), "LineWidth", 1.5); hold on;
    if any(gnss_update & mask)
        scatter(t(mask & gnss_update), delta_vel_xy(mask & gnss_update), 18, "filled");
    end
    yline(0.05, "--", "0.05 m/s");
    yline(0.10, ":", "0.10 m/s");
else
    text(0.1, 0.5, "delta_vel_gnss_update_n/e columns not found", "Units", "normalized");
end
grid on;
xlabel("Time [s]");
ylabel("Delta velocity [m/s]");
title("GNSS Update Velocity Correction Magnitude");

nexttile;
if has_delta_pos
    plot(t(mask), delta_pos_xy(mask), "LineWidth", 1.5); hold on;
    if any(gnss_update & mask)
        scatter(t(mask & gnss_update), delta_pos_xy(mask & gnss_update), 18, "filled");
    end
    yline(0.05, "--", "0.05 m");
    yline(0.10, ":", "0.10 m");
else
    text(0.1, 0.5, "delta_pos_gnss_update_n/e columns not found", "Units", "normalized");
end
grid on;
xlabel("Time [s]");
ylabel("Delta position [m]");
title("GNSS Update Position Correction Magnitude");

%% ============================================================
%  15. Figure 5: Optional PWM vs EKF Stability
% ============================================================
if has_pwm
    figure("Name", "PWM vs EKF XY Stability", "Color", "w");
    tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

    nexttile;
    plot(t(mask), pwm_mean(mask), "LineWidth", 1.5);
    grid on;
    xlabel("Time [s]");
    ylabel("Mean PWM");
    title("Mean Motor PWM");

    nexttile;
    plot(t(mask), vel_xy(mask), "LineWidth", 1.5);
    grid on;
    xlabel("Time [s]");
    ylabel("v_{xy} [m/s]");
    title("EKF XY Velocity Magnitude");

    nexttile;
    plot(t(mask), pos_radius(mask), "LineWidth", 1.5);
    grid on;
    xlabel("Time [s]");
    ylabel("XY radius [m]");
    title("EKF XY Position Scatter");
end