clear; clc; close all;

%% ============================================================
% STRIX FC CSV Accel Quality Analyzer
% =============================================================
% 현재 Data*.csv에 있는 값만 사용해서 다음을 분석합니다.
% 1) raw accel 자체가 PWM에서 튀는지
% 2) LPF+bias 제거 후 ekf_accel_body에서 얼마나 줄었는지
% 3) NED 회전+gravity 보정 후 acc_ned가 propagation에 얼마나 크게 들어가는지
% 4) EKF velocity spike가 acc_ned / raw accel / PWM과 동기화되는지
% 5) GNSS velocity update가 spike 이후 velocity를 얼마나 잡는지
% 6) 가속도 외란 구간과 일반 구간을 분리해서 성능 비교

%% =========================
% 0. User Settings
% =========================

csv_file = "260610\fc_damp_04.CSV";
% csv_file = "data\vel_test_03.CSV";

eval_start_sec = 0.0;
eval_end_sec   = inf;

min_fix_type = 3;
min_num_sats = 6;
use_ekf_ready = true;
use_gnss_ref_ready = true;

pwm_high_threshold = 1400.0;       % 고출력 판단 PWM
acc_h_threshold    = 3.0;          % acc_ned horizontal 기준 [m/s^2]
gnss_speed_low_th  = 0.3;          % GNSS 정지 판단 [m/s]
raw_acc_norm_th_g  = 2.0;          % raw accel norm 외란 기준 [g]
speed_spike_th     = 0.5;          % velocity spike 기준 [m/s]

zoom_before = 5.0;
zoom_after  = 5.0;

g0 = 9.80665;

%% =========================
% 1. Load
% =========================

if ~isfile(csv_file)
    error("CSV 파일을 찾을 수 없습니다: %s", csv_file);
end

T = readtable(csv_file, "VariableNamingRule", "preserve");
cols = string(T.Properties.VariableNames);
N = height(T);

fprintf("\n=================================================\n");
fprintf("STRIX FC CSV Accel Quality Analyzer\n");
fprintf("=================================================\n");
fprintf("Loaded CSV : %s\n", csv_file);
fprintf("Rows       : %d\n", N);
fprintf("Columns    : %d\n", width(T));

need(cols, "timestamp_ms");

t = double(T.timestamp_ms) * 1e-3;
t = t - t(1);
eval_mask = t >= eval_start_sec & t <= eval_end_sec;

%% =========================
% 2. Load Required Data
% =========================

need(cols, ["ax","ay","az"]);
need(cols, ["ekf_accel_body_x","ekf_accel_body_y","ekf_accel_body_z"]);
need(cols, ["acc_ned_n","acc_ned_e","acc_ned_d"]);
need(cols, ["ekf_vel_n","ekf_vel_e","ekf_vel_d"]);
need(cols, ["gnss_vel_n_mps","gnss_vel_e_mps","gnss_vel_d_mps"]);
need(cols, ["ekf_pos_n","ekf_pos_e","ekf_pos_d"]);
need(cols, ["ekf_gnss_pos_n","ekf_gnss_pos_e","ekf_gnss_pos_d"]);

raw_acc_g = [double(T.ax), double(T.ay), double(T.az)];
raw_acc_mps2 = raw_acc_g * g0;
raw_acc_norm_g = sqrt(sum(raw_acc_g.^2, 2));
raw_acc_norm_mps2 = raw_acc_norm_g * g0;

acc_body = [double(T.ekf_accel_body_x), double(T.ekf_accel_body_y), double(T.ekf_accel_body_z)];
acc_body_norm = sqrt(sum(acc_body.^2, 2));

acc_ned = [double(T.acc_ned_n), double(T.acc_ned_e), double(T.acc_ned_d)];
acc_ned_h = hypot(acc_ned(:,1), acc_ned(:,2));
acc_ned_norm = sqrt(sum(acc_ned.^2, 2));

ekf_vel = [double(T.ekf_vel_n), double(T.ekf_vel_e), double(T.ekf_vel_d)];
ekf_speed_h = hypot(ekf_vel(:,1), ekf_vel(:,2));

gnss_vel = [double(T.gnss_vel_n_mps), double(T.gnss_vel_e_mps), double(T.gnss_vel_d_mps)];
gnss_speed_h = hypot(gnss_vel(:,1), gnss_vel(:,2));
vel_diff = gnss_vel - ekf_vel;
vel_diff_h = hypot(vel_diff(:,1), vel_diff(:,2));

ekf_pos = [double(T.ekf_pos_n), double(T.ekf_pos_e), double(T.ekf_pos_d)];
gnss_pos = [double(T.ekf_gnss_pos_n), double(T.ekf_gnss_pos_e), double(T.ekf_gnss_pos_d)];
pos_err = gnss_pos - ekf_pos;
pos_err_h = hypot(pos_err(:,1), pos_err(:,2));

gnss_hacc = col(T, cols, "gnss_hacc_m", nan(N,1));
gnss_sacc = col(T, cols, "gnss_sacc_mps", nan(N,1));

if ismember("ekf_pwm_mean", cols)
    pwm = double(T.ekf_pwm_mean);
elseif all(ismember(["M1","M2","M3","M4"], cols))
    pwm = mean([double(T.M1), double(T.M2), double(T.M3), double(T.M4)], 2, "omitnan");
else
    pwm = nan(N,1);
    warning("PWM 컬럼을 찾지 못했습니다.");
end

dpwm_dt = grad_safe(pwm, t);

roll_deg  = col(T, cols, "roll_deg", col(T, cols, "ekf_roll_deg", nan(N,1)));
pitch_deg = col(T, cols, "pitch_deg", col(T, cols, "ekf_pitch_deg", nan(N,1)));
yaw_deg   = col(T, cols, "yaw_deg", col(T, cols, "ekf_yaw_deg", nan(N,1)));

innov_vel = [
    col(T, cols, "ekf_innov_vel_n", nan(N,1)), ...
    col(T, cols, "ekf_innov_vel_e", nan(N,1)), ...
    col(T, cols, "ekf_innov_vel_d", nan(N,1))
];
innov_vel_h = hypot(innov_vel(:,1), innov_vel(:,2));

R_vel = [
    col(T, cols, "ekf_R_applied_gnss_vel_n", nan(N,1)), ...
    col(T, cols, "ekf_R_applied_gnss_vel_e", nan(N,1)), ...
    col(T, cols, "ekf_R_applied_gnss_vel_d", nan(N,1))
];
sigma_vel_h = sqrt(max(0.5 * (R_vel(:,1) + R_vel(:,2)), 0));

delta_vel_gnss = [
    col(T, cols, "delta_vel_gnss_update_n", zeros(N,1)), ...
    col(T, cols, "delta_vel_gnss_update_e", zeros(N,1)), ...
    col(T, cols, "delta_vel_gnss_update_d", zeros(N,1))
];
delta_vel_gnss_h = hypot(delta_vel_gnss(:,1), delta_vel_gnss(:,2));

delta_vel_zupt = [
    col(T, cols, "delta_vel_zupt_update_n", zeros(N,1)), ...
    col(T, cols, "delta_vel_zupt_update_e", zeros(N,1)), ...
    col(T, cols, "delta_vel_zupt_update_d", zeros(N,1))
];
delta_vel_zupt_h = hypot(delta_vel_zupt(:,1), delta_vel_zupt(:,2));

eff_gain_vel_h = delta_vel_gnss_h ./ max(innov_vel_h, 1e-9);
eff_gain_vel_h(~isfinite(eff_gain_vel_h)) = NaN;

gnss_update_executed = boolcol(T, cols, "gnss_update_executed", false(N,1));
gnss_accepted = boolcol(T, cols, "gnss_correction_accepted", false(N,1));
gnss_velocity_used = boolcol(T, cols, "gnss_velocity_used", false(N,1));

zupt_active = boolcol(T, cols, "zupt_active", false(N,1));
stationary_detected = boolcol(T, cols, "stationary_detected", false(N,1));
ekf_dt = col(T, cols, "ekf_dt", nan(N,1));
ekf_dt_clamped = boolcol(T, cols, "ekf_dt_clamped", false(N,1));

accel_bias = [
    col(T, cols, "ekf_bias_acc_x", col(T, cols, "accel_bias_x", nan(N,1))), ...
    col(T, cols, "ekf_bias_acc_y", col(T, cols, "accel_bias_y", nan(N,1))), ...
    col(T, cols, "ekf_bias_acc_z", col(T, cols, "accel_bias_z", nan(N,1)))
];

gyro_bias = [
    col(T, cols, "ekf_bias_gyro_x", col(T, cols, "gyro_bias_x", nan(N,1))), ...
    col(T, cols, "ekf_bias_gyro_y", col(T, cols, "gyro_bias_y", nan(N,1))), ...
    col(T, cols, "ekf_bias_gyro_z", col(T, cols, "gyro_bias_z", nan(N,1)))
];

v_cov = [
    col(T, cols, "ekf_v_cov_n", nan(N,1)), ...
    col(T, cols, "ekf_v_cov_e", nan(N,1)), ...
    col(T, cols, "ekf_v_cov_d", nan(N,1))
];
sigma_P_vel_h = sqrt(max(0.5 * (v_cov(:,1) + v_cov(:,2)), 0));

%% =========================
% 3. Valid Mask
% =========================

gnss_valid = all(isfinite(gnss_pos), 2);

if ismember("gnss_valid", cols)
    gnss_valid = gnss_valid & double(T.gnss_valid) ~= 0;
end

if ismember("gnss_fix_type", cols)
    gnss_valid = gnss_valid & double(T.gnss_fix_type) >= min_fix_type;
end

if ismember("numSV", cols)
    gnss_valid = gnss_valid & double(T.numSV) >= min_num_sats;
elseif ismember("gnss_num_sats", cols)
    gnss_valid = gnss_valid & double(T.gnss_num_sats) >= min_num_sats;
end

if use_gnss_ref_ready && ismember("gnss_ref_ready", cols)
    gnss_valid = gnss_valid & double(T.gnss_ref_ready) ~= 0;
end

gnss_valid = gnss_valid & vecnorm(gnss_pos, 2, 2) > 1e-9;

ekf_valid = all(isfinite(ekf_pos), 2) & all(isfinite(ekf_vel), 2);

if use_ekf_ready && ismember("ekf_ready", cols)
    ekf_valid = ekf_valid & double(T.ekf_ready) ~= 0;
end

mask = eval_mask & gnss_valid & ekf_valid;

if sum(mask) < 5
    error("평가 가능한 샘플이 너무 적습니다. mask count = %d", sum(mask));
end

%% =========================
% 4. Disturbance Classification
% =========================
% acc_disturbance:
%   줄인지 아닌지 상관없이 가속도계 입력 또는 propagation 가속도가 큰 구간
%
% high_pwm_acc_disturbance:
%   PWM이 높고 GNSS 속도는 거의 0인데 acc_ned가 큰 구간
%   줄 장력/구속/고출력 진동 의심 구간

acc_disturbance = (acc_ned_h > acc_h_threshold) | (raw_acc_norm_g > raw_acc_norm_th_g);

high_pwm_acc_disturbance = (pwm > pwm_high_threshold) & ...
                           (acc_ned_h > acc_h_threshold) & ...
                           (gnss_speed_h < gnss_speed_low_th);

tether_candidate_csv = boolcol(T, cols, "tether_disturbance_candidate", false(N,1));

normal_mask = mask & ~acc_disturbance;
acc_dist_mask = mask & acc_disturbance;
high_pwm_dist_mask = mask & high_pwm_acc_disturbance;

%% =========================
% 5. Spike Detection
% =========================

spike_candidates = mask & ekf_speed_h >= speed_spike_th;

if any(spike_candidates)
    tmp = ekf_speed_h;
    tmp(~spike_candidates) = -inf;
    [~, spike_idx] = max(tmp);
else
    tmp = ekf_speed_h;
    tmp(~mask) = -inf;
    [~, spike_idx] = max(tmp);
end

spike_time = t(spike_idx);
zoom_mask = t >= spike_time - zoom_before & t <= spike_time + zoom_after;

%% =========================
% 6. Console Summary
% =========================

fprintf("\n=================================================\n");
fprintf("Evaluation Summary\n");
fprintf("=================================================\n");
fprintf("Evaluation samples              : %d\n", sum(mask));
fprintf("Evaluation time range           : %.3f ~ %.3f sec\n", min(t(mask)), max(t(mask)));
fprintf("Normal samples                  : %d\n", sum(normal_mask));
fprintf("acc_disturbance samples         : %d\n", sum(acc_dist_mask));
fprintf("high_pwm_acc_disturbance samples: %d\n", sum(high_pwm_dist_mask));
fprintf("GNSS hAcc mean/median/max       : %.4f / %.4f / %.4f m\n", meanf(gnss_hacc(mask)), medianf(gnss_hacc(mask)), maxf(gnss_hacc(mask)));
fprintf("GNSS sAcc mean/median/max       : %.4f / %.4f / %.4f m/s\n", meanf(gnss_sacc(mask)), medianf(gnss_sacc(mask)), maxf(gnss_sacc(mask)));
fprintf("sigma_vel_H mean/max            : %.4f / %.4f m/s\n", meanf(sigma_vel_h(mask)), maxf(sigma_vel_h(mask)));
fprintf("sqrt(P_vel_H) mean/max          : %.4f / %.4f\n", meanf(sigma_P_vel_h(mask)), maxf(sigma_P_vel_h(mask)));
fprintf("GNSS update executed count      : %d\n", sum(gnss_update_executed & mask));
fprintf("GNSS accepted count             : %d\n", sum(gnss_accepted & mask));
fprintf("dt clamped count                : %d\n", sum(ekf_dt_clamped & mask));

print_stats("ALL", mask, pos_err_h, ekf_speed_h, vel_diff_h, raw_acc_norm_g, acc_body_norm, acc_ned_h);
print_stats("NORMAL: no accel disturbance", normal_mask, pos_err_h, ekf_speed_h, vel_diff_h, raw_acc_norm_g, acc_body_norm, acc_ned_h);
print_stats("ACC DISTURBANCE", acc_dist_mask, pos_err_h, ekf_speed_h, vel_diff_h, raw_acc_norm_g, acc_body_norm, acc_ned_h);
print_stats("HIGH PWM ACC DISTURBANCE", high_pwm_dist_mask, pos_err_h, ekf_speed_h, vel_diff_h, raw_acc_norm_g, acc_body_norm, acc_ned_h);

fprintf("\n=================================================\n");
fprintf("Max Velocity Spike\n");
fprintf("=================================================\n");
fprintf("time                             : %.3f sec\n", spike_time);
fprintf("PWM                              : %.2f\n", pwm(spike_idx));
fprintf("EKF speed H                      : %.4f m/s\n", ekf_speed_h(spike_idx));
fprintf("GNSS speed H                     : %.4f m/s\n", gnss_speed_h(spike_idx));
fprintf("GNSS-EKF velocity diff H         : %.4f m/s\n", vel_diff_h(spike_idx));
fprintf("raw accel norm                   : %.4f g\n", raw_acc_norm_g(spike_idx));
fprintf("ekf_accel_body norm              : %.4f m/s^2\n", acc_body_norm(spike_idx));
fprintf("acc_ned H                        : %.4f m/s^2\n", acc_ned_h(spike_idx));
fprintf("acc_ned N/E/D                    : %.4f / %.4f / %.4f m/s^2\n", acc_ned(spike_idx,1), acc_ned(spike_idx,2), acc_ned(spike_idx,3));
fprintf("roll/pitch/yaw                   : %.3f / %.3f / %.3f deg\n", roll_deg(spike_idx), pitch_deg(spike_idx), yaw_deg(spike_idx));
fprintf("GNSS update/accepted/velused     : %d / %d / %d\n", gnss_update_executed(spike_idx), gnss_accepted(spike_idx), gnss_velocity_used(spike_idx));
fprintf("innov_vel_H / sigma_vel_H        : %.4f / %.4f m/s\n", innov_vel_h(spike_idx), sigma_vel_h(spike_idx));
fprintf("dV_GNSS_H / dV_ZUPT_H            : %.4f / %.4f m/s\n", delta_vel_gnss_h(spike_idx), delta_vel_zupt_h(spike_idx));
fprintf("effective gain approx            : %.4f\n", eff_gain_vel_h(spike_idx));
fprintf("acc_disturbance                  : %d\n", acc_disturbance(spike_idx));
fprintf("high_pwm_acc_disturbance         : %d\n", high_pwm_acc_disturbance(spike_idx));
fprintf("tether_candidate_csv             : %d\n", tether_candidate_csv(spike_idx));
fprintf("ekf_dt / dt_clamped              : %.6f / %d\n", ekf_dt(spike_idx), ekf_dt_clamped(spike_idx));
fprintf("=================================================\n\n");

%% =========================
% 7. Figure 1: Accel Path Overview
% =========================

figure("Name", "01 Accel Path Overview");
set(gcf, "Color", "w");

subplot(6,1,1); hold on; grid on;
plot(t, pwm, "LineWidth", 1.1);
xline(spike_time, ":");
ylabel("PWM"); title("PWM");

subplot(6,1,2); hold on; grid on;
plot(t, raw_acc_norm_g, "LineWidth", 1.0);
yline(1.0, "--"); yline(raw_acc_norm_th_g, ":"); xline(spike_time, ":");
ylabel("|raw acc| [g]"); title("Raw accel norm before LPF/bias removal");

subplot(6,1,3); hold on; grid on;
plot(t, acc_body_norm, "LineWidth", 1.0);
xline(spike_time, ":");
ylabel("|body acc|"); title("ekf_accel_body norm after LPF/bias removal [m/s²]");

subplot(6,1,4); hold on; grid on;
plot(t, acc_ned_h, "LineWidth", 1.0);
yline(acc_h_threshold, ":"); xline(spike_time, ":");
ylabel("acc H"); title("acc_ned horizontal, actual propagation input [m/s²]");

subplot(6,1,5); hold on; grid on;
plot(t, ekf_speed_h, "LineWidth", 1.0);
plot(t, gnss_speed_h, "--", "LineWidth", 1.0);
yline(speed_spike_th, ":"); xline(spike_time, ":");
ylabel("Speed H"); title("EKF speed vs GNSS speed [m/s]");
legend("EKF", "GNSS", "spike th", "spike", "Location", "best");

subplot(6,1,6); hold on; grid on;
stairs(t, double(acc_disturbance), "LineWidth", 1.0);
stairs(t, double(high_pwm_acc_disturbance), "LineWidth", 1.0);
stairs(t, double(gnss_update_executed), "LineWidth", 0.8);
stairs(t, double(zupt_active), "LineWidth", 1.0);
xline(spike_time, ":"); ylim([-0.1, 1.1]);
ylabel("flag"); xlabel("Time [s]");
title("Disturbance / GNSS update / ZUPT");
legend("acc disturbance", "high PWM acc disturbance", "GNSS update", "ZUPT", "Location", "best");

%% =========================
% 8. Figure 2: Accel Axes Through Pipeline
% =========================

figure("Name", "02 Accel Axes Through Pipeline");
set(gcf, "Color", "w");

subplot(5,1,1); hold on; grid on;
plot(t, pwm, "LineWidth", 1.0); xline(spike_time, ":");
ylabel("PWM"); title("PWM");

subplot(5,1,2); hold on; grid on;
plot(t, raw_acc_mps2(:,1), "LineWidth", 1.0);
plot(t, raw_acc_mps2(:,2), "LineWidth", 1.0);
plot(t, raw_acc_mps2(:,3), "LineWidth", 1.0);
xline(spike_time, ":");
ylabel("[m/s²]"); title("Raw accel ax/ay/az converted to m/s²");
legend("ax raw", "ay raw", "az raw", "Location", "best");

subplot(5,1,3); hold on; grid on;
plot(t, acc_body(:,1), "LineWidth", 1.0);
plot(t, acc_body(:,2), "LineWidth", 1.0);
plot(t, acc_body(:,3), "LineWidth", 1.0);
xline(spike_time, ":");
ylabel("[m/s²]"); title("ekf_accel_body x/y/z after LPF and bias removal");
legend("body x", "body y", "body z", "Location", "best");

subplot(5,1,4); hold on; grid on;
plot(t, acc_ned(:,1), "LineWidth", 1.0);
plot(t, acc_ned(:,2), "LineWidth", 1.0);
plot(t, acc_ned(:,3), "LineWidth", 1.0);
xline(spike_time, ":");
ylabel("[m/s²]"); title("acc_ned N/E/D after attitude rotation and gravity compensation");
legend("N", "E", "D", "Location", "best");

subplot(5,1,5); hold on; grid on;
plot(t, roll_deg, "LineWidth", 1.0);
plot(t, pitch_deg, "LineWidth", 1.0);
xline(spike_time, ":");
ylabel("[deg]"); xlabel("Time [s]"); title("Roll / Pitch");
legend("roll", "pitch", "Location", "best");

%% =========================
% 9. Figure 3: GNSS Velocity Update
% =========================

figure("Name", "03 GNSS Velocity Update");
set(gcf, "Color", "w");

subplot(6,1,1); hold on; grid on;
plot(t, ekf_vel(:,1), "LineWidth", 1.0);
plot(t, gnss_vel(:,1), "--", "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("V_N"); title("Velocity N [m/s]");
legend("EKF", "GNSS", "Location", "best");

subplot(6,1,2); hold on; grid on;
plot(t, ekf_vel(:,2), "LineWidth", 1.0);
plot(t, gnss_vel(:,2), "--", "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("V_E"); title("Velocity E [m/s]");
legend("EKF", "GNSS", "Location", "best");

subplot(6,1,3); hold on; grid on;
plot(t, innov_vel_h, "LineWidth", 1.0);
plot(t, sigma_vel_h, "--", "LineWidth", 1.0);
plot(t, gnss_sacc, ":", "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("[m/s]");
title("Velocity innovation H vs sigma R vel H and sAcc");
legend("innov vel H", "sigma R vel H", "sAcc", "Location", "best");

subplot(6,1,4); hold on; grid on;
plot(t, delta_vel_gnss_h, "LineWidth", 1.0);
plot(t, delta_vel_zupt_h, "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("[m/s]"); title("Correction delta velocity");
legend("dV GNSS H", "dV ZUPT H", "Location", "best");

subplot(6,1,5); hold on; grid on;
plot(t, eff_gain_vel_h, "LineWidth", 1.0);
yline(1.0, "--"); xline(spike_time, ":");
ylabel("ratio"); title("|dV_GNSS| / |innov_vel| approximation");

subplot(6,1,6); hold on; grid on;
stairs(t, double(gnss_update_executed), "LineWidth", 1.0);
stairs(t, double(gnss_accepted), "LineWidth", 1.0);
stairs(t, double(gnss_velocity_used), "LineWidth", 1.0);
xline(spike_time, ":"); ylim([-0.1, 1.1]);
ylabel("flag"); xlabel("Time [s]"); title("GNSS update flags");
legend("executed", "accepted", "vel used", "Location", "best");

%% =========================
% 10. Figure 4: Bias / Covariance
% =========================

figure("Name", "04 Bias and Covariance");
set(gcf, "Color", "w");

subplot(5,1,1); hold on; grid on;
plot(t, pwm, "LineWidth", 1.0); xline(spike_time, ":");
ylabel("PWM"); title("PWM");

subplot(5,1,2); hold on; grid on;
plot(t, accel_bias(:,1), "LineWidth", 1.0);
plot(t, accel_bias(:,2), "LineWidth", 1.0);
plot(t, accel_bias(:,3), "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("acc bias"); title("Accelerometer bias estimate");
legend("b_ax", "b_ay", "b_az", "Location", "best");

subplot(5,1,3); hold on; grid on;
plot(t, gyro_bias(:,1), "LineWidth", 1.0);
plot(t, gyro_bias(:,2), "LineWidth", 1.0);
plot(t, gyro_bias(:,3), "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("gyro bias"); title("Gyro bias estimate");
legend("b_gx", "b_gy", "b_gz", "Location", "best");

subplot(5,1,4); hold on; grid on;
plot(t, sigma_P_vel_h, "LineWidth", 1.0);
plot(t, sigma_vel_h, "--", "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("[m/s]"); title("sqrt(P_vel_H) vs sigma_R_vel_H");
legend("sqrt(P vel H)", "sigma R vel H", "Location", "best");

subplot(5,1,5); hold on; grid on;
plot(t, vel_diff_h, "LineWidth", 1.0);
plot(t, acc_ned_h, "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("diff / acc"); xlabel("Time [s]");
title("GNSS-EKF velocity diff H vs acc_ned H");
legend("|V_GNSS - V_EKF|", "acc_ned H", "Location", "best");

%% =========================
% 11. Figure 5: Spike Zoom
% =========================

figure("Name", "05 Spike Zoom");
set(gcf, "Color", "w");

subplot(8,1,1); hold on; grid on;
plot(t(zoom_mask), pwm(zoom_mask), "LineWidth", 1.1);
xline(spike_time, ":"); ylabel("PWM");
title(sprintf("Spike Zoom around %.3f sec", spike_time));

subplot(8,1,2); hold on; grid on;
plot(t(zoom_mask), raw_acc_norm_g(zoom_mask), "LineWidth", 1.0);
yline(1.0, "--"); yline(raw_acc_norm_th_g, ":"); xline(spike_time, ":");
ylabel("|raw| [g]"); title("Raw accel norm");

subplot(8,1,3); hold on; grid on;
plot(t(zoom_mask), acc_body_norm(zoom_mask), "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("|body|"); title("ekf_accel_body norm");

subplot(8,1,4); hold on; grid on;
plot(t(zoom_mask), acc_ned(zoom_mask,1), "LineWidth", 1.0);
plot(t(zoom_mask), acc_ned(zoom_mask,2), "LineWidth", 1.0);
plot(t(zoom_mask), acc_ned_h(zoom_mask), "LineWidth", 1.1);
xline(spike_time, ":"); ylabel("acc"); title("acc_ned N/E/H");
legend("N", "E", "H", "Location", "best");

subplot(8,1,5); hold on; grid on;
plot(t(zoom_mask), ekf_speed_h(zoom_mask), "LineWidth", 1.1);
plot(t(zoom_mask), gnss_speed_h(zoom_mask), "--", "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("speed"); title("EKF/GNSS horizontal speed");
legend("EKF", "GNSS", "Location", "best");

subplot(8,1,6); hold on; grid on;
plot(t(zoom_mask), innov_vel_h(zoom_mask), "LineWidth", 1.0);
plot(t(zoom_mask), delta_vel_gnss_h(zoom_mask), "LineWidth", 1.0);
plot(t(zoom_mask), sigma_vel_h(zoom_mask), "--", "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("[m/s]"); title("GNSS velocity update diagnostic");
legend("innov vel H", "dV GNSS H", "sigma R vel H", "Location", "best");

subplot(8,1,7); hold on; grid on;
plot(t(zoom_mask), roll_deg(zoom_mask), "LineWidth", 1.0);
plot(t(zoom_mask), pitch_deg(zoom_mask), "LineWidth", 1.0);
xline(spike_time, ":"); ylabel("[deg]"); title("Roll / Pitch");
legend("roll", "pitch", "Location", "best");

subplot(8,1,8); hold on; grid on;
stairs(t(zoom_mask), double(acc_disturbance(zoom_mask)), "LineWidth", 1.0);
stairs(t(zoom_mask), double(high_pwm_acc_disturbance(zoom_mask)), "LineWidth", 1.0);
stairs(t(zoom_mask), double(gnss_update_executed(zoom_mask)), "LineWidth", 1.0);
stairs(t(zoom_mask), double(zupt_active(zoom_mask)), "LineWidth", 1.0);
xline(spike_time, ":"); ylim([-0.1, 1.1]);
ylabel("flag"); xlabel("Time [s]"); title("Flags");
legend("acc disturbance", "high PWM acc disturbance", "GNSS update", "ZUPT", "Location", "best");

%% =========================
% 12. Figure 6: Normal vs Disturbance
% =========================

figure("Name", "06 Normal vs Acc Disturbance");
set(gcf, "Color", "w");

subplot(4,1,1); hold on; grid on;
histogram(ekf_speed_h(normal_mask), 30);
histogram(ekf_speed_h(acc_dist_mask), 30);
xlabel("EKF speed H [m/s]"); ylabel("count"); title("EKF speed histogram");
legend("normal", "acc disturbance");

subplot(4,1,2); hold on; grid on;
histogram(raw_acc_norm_g(normal_mask), 30);
histogram(raw_acc_norm_g(acc_dist_mask), 30);
xlabel("raw accel norm [g]"); ylabel("count"); title("Raw accel norm histogram");
legend("normal", "acc disturbance");

subplot(4,1,3); hold on; grid on;
histogram(acc_ned_h(normal_mask), 30);
histogram(acc_ned_h(acc_dist_mask), 30);
xlabel("acc_ned H [m/s²]"); ylabel("count"); title("acc_ned horizontal histogram");
legend("normal", "acc disturbance");

subplot(4,1,4); hold on; grid on;
plot(t(mask), pos_err_h(mask), "LineWidth", 1.0);
plot(t(mask), gnss_hacc(mask), "--", "LineWidth", 1.0);
xline(spike_time, ":");
xlabel("Time [s]"); ylabel("[m]"); title("Position error vs hAcc");
legend("H error", "hAcc", "spike", "Location", "best");

%% =========================
% 13. Save Report
% =========================

report = struct();
report.csv_file = csv_file;
report.t = t;
report.mask = mask;
report.normal_mask = normal_mask;
report.acc_dist_mask = acc_dist_mask;
report.high_pwm_dist_mask = high_pwm_dist_mask;
report.spike_idx = spike_idx;
report.spike_time = spike_time;

report.pwm = pwm;
report.raw_acc_g = raw_acc_g;
report.raw_acc_norm_g = raw_acc_norm_g;
report.acc_body = acc_body;
report.acc_body_norm = acc_body_norm;
report.acc_ned = acc_ned;
report.acc_ned_h = acc_ned_h;
report.ekf_vel = ekf_vel;
report.ekf_speed_h = ekf_speed_h;
report.gnss_vel = gnss_vel;
report.gnss_speed_h = gnss_speed_h;
report.vel_diff_h = vel_diff_h;
report.innov_vel_h = innov_vel_h;
report.sigma_vel_h = sigma_vel_h;
report.delta_vel_gnss_h = delta_vel_gnss_h;
report.eff_gain_vel_h = eff_gain_vel_h;
report.accel_bias = accel_bias;
report.gyro_bias = gyro_bias;
report.pos_err_h = pos_err_h;
report.acc_disturbance = acc_disturbance;
report.high_pwm_acc_disturbance = high_pwm_acc_disturbance;

% save("strix_current_csv_accel_quality_report.mat", "report");
% fprintf("Saved report: strix_current_csv_accel_quality_report.mat\n");

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

function x = boolcol(T, cols, name, default_value)
if ismember(name, cols)
    x = double(T.(name)) ~= 0;
else
    x = default_value;
end
end

function y = rmsf(x)
x = x(:);
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = sqrt(mean(x.^2));
end
end

function y = meanf(x)
x = x(:);
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = mean(x);
end
end

function y = medianf(x)
x = x(:);
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = median(x);
end
end

function y = maxf(x)
x = x(:);
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = max(x);
end
end

function dxdt = grad_safe(x, t)
x = x(:);
t = t(:);
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

function print_stats(name, m, pos_err_h, ekf_speed_h, vel_diff_h, raw_acc_norm_g, acc_body_norm, acc_ned_h)
fprintf("\n[%s]\n", name);
fprintf("samples                  : %d\n", sum(m));
if sum(m) < 5
    fprintf("Not enough samples.\n");
    return;
end
fprintf("Position H RMSE/Max      : %.4f / %.4f m\n", rmsf(pos_err_h(m)), maxf(pos_err_h(m)));
fprintf("EKF speed H mean/max     : %.4f / %.4f m/s\n", meanf(ekf_speed_h(m)), maxf(ekf_speed_h(m)));
fprintf("GNSS-EKF dV H mean/max   : %.4f / %.4f m/s\n", meanf(vel_diff_h(m)), maxf(vel_diff_h(m)));
fprintf("raw accel norm mean/max  : %.4f / %.4f g\n", meanf(raw_acc_norm_g(m)), maxf(raw_acc_norm_g(m)));
fprintf("body accel norm mean/max : %.4f / %.4f m/s^2\n", meanf(acc_body_norm(m)), maxf(acc_body_norm(m)));
fprintf("acc_ned H mean/max       : %.4f / %.4f m/s^2\n", meanf(acc_ned_h(m)), maxf(acc_ned_h(m)));
end
