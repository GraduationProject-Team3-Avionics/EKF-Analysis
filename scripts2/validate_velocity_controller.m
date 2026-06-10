%% STRIX FC - Velocity Controller Validation with GNSS velocity reference + fixed attitude
% CSV: vel_ctrl_1.CSV
%
% 목적:
%   1) 속도 제어기 ON/VALID 구간만 평가
%   2) EKF/filtered velocity와 GNSS velocity를 같이 비교
%   3) GNSS 속도 신뢰도(sAcc, applied R/sigma, update 사용 여부)를 같이 확인
%   4) EBIMU quaternion(qw/qx/qy/qz) 기반 자세와 EKF 자세를 roll/pitch/yaw 한 figure에서 비교
%   5) desired roll/pitch와 EBIMU/EKF 실제 roll/pitch를 같은 plot에서 비교

clear; clc; close all;

%% ===================== User setting =====================
csv_file = "data/vel_ctrl_1.CSV";

USE_ONLY_VALID_CTRL = true;     % true: vel_ctrl_enabled && vel_ctrl_valid 구간만 평가
USE_FILTERED_VEL    = true;     % true: vel_filt_* 기준, false: vel_meas_* 기준

% GNSS를 reference로 볼 때의 최소 조건
MIN_GNSS_SATS       = 6;
MAX_GNSS_SACC_MPS   = 0.30;     % sAcc가 이 값보다 작으면 신뢰 가능 후보로 판단
USE_ONLY_NEW_GNSS   = false;    % true: gnss_new_measurement==1인 샘플만 GNSS reference 비교

%% ===================== Load =====================
T = readtable(csv_file, "VariableNamingRule", "preserve");
vars = string(T.Properties.VariableNames);

required = [
    "timestamp_ms"
    "vel_ctrl_enabled"
    "vel_ctrl_valid"
    "vel_sp_n_mps"
    "vel_sp_e_mps"
    "vel_meas_n_mps"
    "vel_meas_e_mps"
    "vel_filt_n_mps"
    "vel_filt_e_mps"
    "vel_acc_cmd_n_mps2"
    "vel_acc_cmd_e_mps2"
    "vel_roll_des_deg"
    "vel_pitch_des_deg"
    "gnss_valid"
    "gnss_num_sats"
    "gnss_vel_n_mps"
    "gnss_vel_e_mps"
    "gnss_sacc_mps"
];

for i = 1:numel(required)
    if ~ismember(required(i), vars)
        error("CSV에 필요한 컬럼이 없습니다: %s", required(i));
    end
end

t = (T.timestamp_ms - T.timestamp_ms(1)) * 1e-3;

ctrl_on    = T.vel_ctrl_enabled == 1;
ctrl_valid = T.vel_ctrl_valid == 1;

if USE_ONLY_VALID_CTRL
    mask_ctrl = ctrl_on & ctrl_valid;
else
    mask_ctrl = true(height(T), 1);
end

if nnz(mask_ctrl) < 2
    error("평가 가능한 vel_ctrl valid 샘플이 너무 적습니다.");
end

idx_start = find(mask_ctrl, 1, "first");
idx_end   = find(mask_ctrl, 1, "last");

%% ===================== Signals =====================
% Setpoint
sp_n = T.vel_sp_n_mps;
sp_e = T.vel_sp_e_mps;
sp_xy = hypot(sp_n, sp_e);

% Controller velocity source
if USE_FILTERED_VEL
    vel_n = T.vel_filt_n_mps;
    vel_e = T.vel_filt_e_mps;
    vel_label = "filtered";
else
    vel_n = T.vel_meas_n_mps;
    vel_e = T.vel_meas_e_mps;
    vel_label = "measured";
end
vel_xy = hypot(vel_n, vel_e);

% GNSS velocity
vn_gnss = T.gnss_vel_n_mps;
ve_gnss = T.gnss_vel_e_mps;
vxy_gnss = hypot(vn_gnss, ve_gnss);

% Error against setpoint
err_n = sp_n - vel_n;
err_e = sp_e - vel_e;
err_xy = hypot(err_n, err_e);

% Difference against GNSS reference
vel_minus_gnss_n  = vel_n - vn_gnss;
vel_minus_gnss_e  = vel_e - ve_gnss;
vel_minus_gnss_xy = hypot(vel_minus_gnss_n, vel_minus_gnss_e);

% Controller output
acc_cmd_n  = T.vel_acc_cmd_n_mps2;
acc_cmd_e  = T.vel_acc_cmd_e_mps2;
acc_cmd_xy = hypot(acc_cmd_n, acc_cmd_e);
roll_des   = T.vel_roll_des_deg;
pitch_des  = T.vel_pitch_des_deg;

%% ===================== Optional columns =====================
has_pwm        = ismember("ekf_pwm_mean", vars);
has_sat_acc    = ismember("vel_ctrl_accel_saturated", vars);
has_sat_tilt   = ismember("vel_ctrl_tilt_saturated", vars);
has_gnss_new   = ismember("gnss_new_measurement", vars);
has_gnss_used  = ismember("gnss_velocity_used", vars);
has_update     = ismember("gnss_update_executed", vars);
has_sigma_vel  = ismember("ekf_sigma_applied_gnss_vel_n", vars) && ismember("ekf_sigma_applied_gnss_vel_e", vars);
has_R_vel      = ismember("ekf_R_applied_gnss_vel_n", vars) && ismember("ekf_R_applied_gnss_vel_e", vars);
has_cov_vel    = ismember("ekf_v_cov_n", vars) && ismember("ekf_v_cov_e", vars);

% Attitude columns
% 중요:
%   이 CSV에서 roll_deg/pitch_deg/yaw_deg는 EBIMU가 아니라 EKF Euler와 같은 값입니다.
%   실제 EBIMU firmware 자세는 qw/qx/qy/qz quaternion 또는 imu_fw_roll_deg/imu_fw_pitch_deg입니다.
%   yaw까지 비교하려면 EBIMU quaternion(qw/qx/qy/qz)에서 직접 RPY를 계산해야 합니다.
has_imu_quat = all(ismember(["qw", "qx", "qy", "qz"], vars));
has_imu_fw_rp = all(ismember(["imu_fw_roll_deg", "imu_fw_pitch_deg"], vars));
has_ekf_quat = all(ismember(["ekf_qw", "ekf_qx", "ekf_qy", "ekf_qz"], vars));
has_ekf_eul  = all(ismember(["ekf_roll_deg", "ekf_pitch_deg", "ekf_yaw_deg"], vars));
has_duplicate_logger_eul = all(ismember(["roll_deg", "pitch_deg", "yaw_deg", "ekf_roll_deg", "ekf_pitch_deg", "ekf_yaw_deg"], vars));

if has_pwm
    pwm_mean = T.ekf_pwm_mean;
end

%% ===================== Attitude signals =====================
% EBIMU: 반드시 firmware quaternion에서 계산한다.
% roll/pitch는 imu_fw_roll_deg / imu_fw_pitch_deg와 거의 같은지 확인용으로도 쓸 수 있다.
if has_imu_quat
    [imu_roll_deg, imu_pitch_deg, imu_yaw_deg] = quat_wxyz_to_euler_deg(T.qw, T.qx, T.qy, T.qz);
    imu_yaw_deg = unwrap_deg(imu_yaw_deg);
elseif has_imu_fw_rp
    imu_roll_deg  = T.imu_fw_roll_deg;
    imu_pitch_deg = T.imu_fw_pitch_deg;
    imu_yaw_deg   = nan(height(T), 1);
    warning("EBIMU quaternion이 없어 yaw는 표시하지 않습니다. roll/pitch만 imu_fw_* 컬럼을 사용합니다.");
else
    warning("EBIMU attitude columns were not found. Attitude figure will be skipped for EBIMU.");
end

% EKF: CSV의 ekf_* Euler를 우선 사용한다. 없으면 EKF quaternion에서 계산한다.
if has_ekf_eul
    ekf_roll_att_deg  = T.ekf_roll_deg;
    ekf_pitch_att_deg = T.ekf_pitch_deg;
    ekf_yaw_att_deg   = unwrap_deg(T.ekf_yaw_deg);
elseif has_ekf_quat
    [ekf_roll_att_deg, ekf_pitch_att_deg, ekf_yaw_att_deg] = quat_wxyz_to_euler_deg(T.ekf_qw, T.ekf_qx, T.ekf_qy, T.ekf_qz);
    ekf_yaw_att_deg = unwrap_deg(ekf_yaw_att_deg);
else
    warning("EKF attitude columns were not found. Attitude figure will be skipped for EKF.");
end

if has_duplicate_logger_eul
    eul_dup_max = max(abs([T.roll_deg - T.ekf_roll_deg; T.pitch_deg - T.ekf_pitch_deg; T.yaw_deg - T.ekf_yaw_deg]), [], "omitnan");
else
    eul_dup_max = NaN;
end

%% ===================== Masks =====================
gnss_quality = ...
    T.gnss_valid == 1 & ...
    T.gnss_num_sats >= MIN_GNSS_SATS & ...
    T.gnss_sacc_mps > 0 & ...
    T.gnss_sacc_mps <= MAX_GNSS_SACC_MPS & ...
    isfinite(vn_gnss) & isfinite(ve_gnss);

if USE_ONLY_NEW_GNSS && has_gnss_new
    gnss_quality = gnss_quality & (T.gnss_new_measurement == 1);
end

m_ctrl = mask_ctrl;
m_gnss_ref = mask_ctrl & gnss_quality;

%% ===================== Metrics =====================
fprintf("\n=================================================\n");
fprintf("STRIX FC XY Velocity Controller Validation + GNSS\n");
fprintf("=================================================\n");
fprintf("CSV                    : %s\n", csv_file);
fprintf("Rows                   : %d\n", height(T));
fprintf("Valid ctrl samples     : %d\n", nnz(m_ctrl));
fprintf("Control time range     : %.3f ~ %.3f sec\n", t(idx_start), t(idx_end));
fprintf("Velocity source        : %s\n", vel_label);
fprintf("GNSS reference samples : %d\n", nnz(m_gnss_ref));
fprintf("GNSS ref condition     : valid=1, sats>=%d, 0<sAcc<=%.2f m/s\n", MIN_GNSS_SATS, MAX_GNSS_SACC_MPS);
if USE_ONLY_NEW_GNSS && has_gnss_new
    fprintf("GNSS ref timing        : only gnss_new_measurement==1\n");
else
    fprintf("GNSS ref timing        : held GNSS velocity included\n");
end
fprintf("Attitude columns       : EBIMU quat=%d, EBIMU fw RP=%d, EKF quat=%d, EKF Euler=%d\n", ...
    has_imu_quat, has_imu_fw_rp, has_ekf_quat, has_ekf_eul);
if isfinite(eul_dup_max)
    fprintf("Logger Euler duplication: max |roll/pitch/yaw - ekf_roll/pitch/yaw| = %.6f deg\n", eul_dup_max);
    fprintf("                         -> roll_deg/pitch_deg/yaw_deg는 EBIMU가 아니라 EKF와 동일하게 로깅된 것으로 판단\n");
end
fprintf("-------------------------------------------------\n");

fprintf("[1] Controller tracking against setpoint\n");
print_metric("vel_sp_xy",    sp_xy(m_ctrl),      "m/s");
print_metric("vel_xy",       vel_xy(m_ctrl),     "m/s");
print_metric("vel_err_xy",   err_xy(m_ctrl),     "m/s");
print_metric("vel_err_n",    err_n(m_ctrl),      "m/s");
print_metric("vel_err_e",    err_e(m_ctrl),      "m/s");

fprintf("-------------------------------------------------\n");
fprintf("[2] GNSS velocity as external reference\n");
print_metric("gnss_vel_xy",       vxy_gnss(m_gnss_ref),          "m/s");
print_metric("vel_minus_gnss_xy", vel_minus_gnss_xy(m_gnss_ref), "m/s");
print_metric("vel_minus_gnss_n",  vel_minus_gnss_n(m_gnss_ref),  "m/s");
print_metric("vel_minus_gnss_e",  vel_minus_gnss_e(m_gnss_ref),  "m/s");

fprintf("-------------------------------------------------\n");
fprintf("[3] Controller output\n");
print_metric("acc_cmd_xy", acc_cmd_xy(m_ctrl), "m/s^2");
print_metric("roll_des",   roll_des(m_ctrl),   "deg");
print_metric("pitch_des",  pitch_des(m_ctrl),  "deg");

fprintf("-------------------------------------------------\n");
fprintf("[4] GNSS reliability inside ctrl-valid section\n");
fprintf("GNSS valid ratio              : %6.2f %%\n", 100 * mean(T.gnss_valid(m_ctrl) == 1));
fprintf("GNSS sats mean / min          : %6.2f / %d\n", mean(T.gnss_num_sats(m_ctrl), "omitnan"), min(T.gnss_num_sats(m_ctrl)));
fprintf("GNSS sAcc mean/median/P95/max : %.4f / %.4f / %.4f / %.4f m/s\n", ...
    mean(T.gnss_sacc_mps(m_ctrl), "omitnan"), ...
    median(T.gnss_sacc_mps(m_ctrl), "omitnan"), ...
    prctile(T.gnss_sacc_mps(m_ctrl), 95), ...
    max(T.gnss_sacc_mps(m_ctrl)));

if has_gnss_new
    fprintf("GNSS new measurement ratio    : %6.2f %%\n", 100 * mean(T.gnss_new_measurement(m_ctrl) == 1));
end
if has_gnss_used
    fprintf("GNSS velocity used ratio      : %6.2f %%\n", 100 * mean(T.gnss_velocity_used(m_ctrl) == 1));
end
if has_update
    fprintf("GNSS update executed ratio    : %6.2f %%\n", 100 * mean(T.gnss_update_executed(m_ctrl) == 1));
end
if has_sigma_vel
    sigma_h = hypot(T.ekf_sigma_applied_gnss_vel_n, T.ekf_sigma_applied_gnss_vel_e);
    fprintf("Applied GNSS vel sigma N/E/H  : %.4f / %.4f / %.4f m/s\n", ...
        mean(T.ekf_sigma_applied_gnss_vel_n(m_ctrl), "omitnan"), ...
        mean(T.ekf_sigma_applied_gnss_vel_e(m_ctrl), "omitnan"), ...
        mean(sigma_h(m_ctrl), "omitnan"));
end
if has_R_vel
    fprintf("Applied GNSS vel R N/E        : %.4f / %.4f (m/s)^2\n", ...
        mean(T.ekf_R_applied_gnss_vel_n(m_ctrl), "omitnan"), ...
        mean(T.ekf_R_applied_gnss_vel_e(m_ctrl), "omitnan"));
end
if has_cov_vel
    ekf_sigma_vel_h = hypot(sqrt(max(T.ekf_v_cov_n,0)), sqrt(max(T.ekf_v_cov_e,0)));
    fprintf("EKF vel sigma H mean/P95      : %.4f / %.4f m/s\n", ...
        mean(ekf_sigma_vel_h(m_ctrl), "omitnan"), prctile(ekf_sigma_vel_h(m_ctrl), 95));
end

fprintf("-------------------------------------------------\n");
fprintf("P(|vel_err_xy| < 0.05 m/s)       : %6.2f %%\n", 100 * mean(err_xy(m_ctrl) < 0.05));
fprintf("P(|vel_err_xy| < 0.10 m/s)       : %6.2f %%\n", 100 * mean(err_xy(m_ctrl) < 0.10));
fprintf("P(|vel_err_xy| < 0.20 m/s)       : %6.2f %%\n", 100 * mean(err_xy(m_ctrl) < 0.20));
if nnz(m_gnss_ref) > 0
    fprintf("P(|vel-GNSS|_xy < 0.10 m/s)      : %6.2f %%\n", 100 * mean(vel_minus_gnss_xy(m_gnss_ref) < 0.10));
    fprintf("P(|vel-GNSS|_xy < 0.20 m/s)      : %6.2f %%\n", 100 * mean(vel_minus_gnss_xy(m_gnss_ref) < 0.20));
end
if has_sat_acc
    fprintf("Accel saturated ratio            : %6.2f %%\n", 100 * mean(T.vel_ctrl_accel_saturated(m_ctrl) == 1));
end
if has_sat_tilt
    fprintf("Tilt saturated ratio             : %6.2f %%\n", 100 * mean(T.vel_ctrl_tilt_saturated(m_ctrl) == 1));
end
fprintf("=================================================\n\n");

%% ===================== Plot 1: N/E velocity tracking + GNSS =====================
figure("Name", "Velocity tracking with GNSS");
tiledlayout(2,1, "TileSpacing", "compact");

nexttile;
plot(t, sp_n, "--", "LineWidth", 1.0); hold on; grid on;
plot(t, vel_n, "LineWidth", 1.2);
plot(t, vn_gnss, ":", "LineWidth", 1.2);
xline(t(idx_start), "--");
xline(t(idx_end), "--");
ylabel("N velocity [m/s]");
legend("sp N", vel_label + " N", "GNSS N", "ctrl start/end", "Location", "best");
title("N-axis velocity tracking");

nexttile;
plot(t, sp_e, "--", "LineWidth", 1.0); hold on; grid on;
plot(t, vel_e, "LineWidth", 1.2);
plot(t, ve_gnss, ":", "LineWidth", 1.2);
xline(t(idx_start), "--");
xline(t(idx_end), "--");
xlabel("Time [s]");
ylabel("E velocity [m/s]");
legend("sp E", vel_label + " E", "GNSS E", "ctrl start/end", "Location", "best");
title("E-axis velocity tracking");

%% ===================== Plot 2: XY speed and difference against GNSS =====================
figure("Name", "XY speed comparison against GNSS");
tiledlayout(2,1, "TileSpacing", "compact");

nexttile;
plot(t, sp_xy, "--", "LineWidth", 1.0); hold on; grid on;
plot(t, vel_xy, "LineWidth", 1.2);
plot(t, vxy_gnss, ":", "LineWidth", 1.2);
xline(t(idx_start), "--");
xline(t(idx_end), "--");
ylabel("XY speed [m/s]");
legend("sp XY", vel_label + " XY", "GNSS XY", "ctrl start/end", "Location", "best");
title("XY speed comparison");

nexttile;
plot(t, err_xy, "LineWidth", 1.2); hold on; grid on;
plot(t, vel_minus_gnss_xy, "LineWidth", 1.2);
xline(t(idx_start), "--");
xline(t(idx_end), "--");
xlabel("Time [s]");
ylabel("Error / difference [m/s]");
legend("|sp - " + vel_label + "|", "|" + vel_label + " - GNSS|", "ctrl start/end", "Location", "best");
title("Setpoint error and GNSS-reference difference");

%% ===================== Plot 3: GNSS velocity reliability =====================
figure("Name", "GNSS velocity reliability");
tiledlayout(2,1, "TileSpacing", "compact");

nexttile;
plot(t, T.gnss_sacc_mps, "LineWidth", 1.2); hold on; grid on;
yline(MAX_GNSS_SACC_MPS, "--");
xline(t(idx_start), "--");
xline(t(idx_end), "--");
ylabel("GNSS sAcc [m/s]");
legend("sAcc", "reference threshold", "ctrl start/end", "Location", "best");
title("GNSS provided speed accuracy");

nexttile;
plot(t, T.gnss_num_sats, "LineWidth", 1.2); hold on; grid on;
yline(MIN_GNSS_SATS, "--");
if has_sigma_vel
    yyaxis right;
    plot(t, T.ekf_sigma_applied_gnss_vel_n, "LineWidth", 1.0);
    plot(t, T.ekf_sigma_applied_gnss_vel_e, "LineWidth", 1.0);
    ylabel("Applied GNSS vel sigma [m/s]");
    yyaxis left;
end
xline(t(idx_start), "--");
xline(t(idx_end), "--");
xlabel("Time [s]");
ylabel("GNSS satellites");
if has_sigma_vel
    legend("num sats", "sat threshold", "sigma vel N", "sigma vel E", "ctrl start/end", "Location", "best");
else
    legend("num sats", "sat threshold", "ctrl start/end", "Location", "best");
end
title("GNSS satellite count and applied velocity covariance");

%% ===================== Plot 4: Controller output =====================
figure("Name", "Velocity controller output");
plot(t, roll_des, "LineWidth", 1.2); hold on; grid on;
plot(t, pitch_des, "LineWidth", 1.2);

if has_pwm
    yyaxis right;
    plot(t, pwm_mean, "LineWidth", 1.0);
    ylabel("PWM mean [us]");
    yyaxis left;
end

xline(t(idx_start), "--");
xline(t(idx_end), "--");
xlabel("Time [s]");
ylabel("Desired attitude [deg]");
title("Velocity controller output: desired roll/pitch");
if has_pwm
    legend("roll des", "pitch des", "PWM mean", "ctrl start/end", "Location", "best");
else
    legend("roll des", "pitch des", "ctrl start/end", "Location", "best");
end

%% ===================== Plot 5: Attitude comparison =====================
if (has_imu_quat || has_imu_fw_rp) && (has_ekf_eul || has_ekf_quat)

    % ctrl start/end 구간만 plot
    idx_plot = idx_start:idx_end;

    t_plot = t(idx_plot);

    imu_roll_plot  = imu_roll_deg(idx_plot);
    ekf_roll_plot  = ekf_roll_att_deg(idx_plot);

    imu_pitch_plot = imu_pitch_deg(idx_plot);
    ekf_pitch_plot = ekf_pitch_att_deg(idx_plot);

    imu_yaw_plot   = imu_yaw_deg(idx_plot);
    ekf_yaw_plot   = ekf_yaw_att_deg(idx_plot);

    figure("Name", "Attitude: EBIMU quaternion vs EKF - control interval only");
    tiledlayout(3,1, "TileSpacing", "compact");

    nexttile;
    plot(t_plot, imu_roll_plot, "LineWidth", 1.1); hold on; grid on;
    plot(t_plot, ekf_roll_plot, "LineWidth", 1.1);
    ylabel("Roll [deg]");
    legend("EBIMU fw", "EKF", "Location", "best");
    title("Roll attitude during velocity control");

    nexttile;
    plot(t_plot, imu_pitch_plot, "LineWidth", 1.1); hold on; grid on;
    plot(t_plot, ekf_pitch_plot, "LineWidth", 1.1);
    ylabel("Pitch [deg]");
    legend("EBIMU fw", "EKF", "Location", "best");
    title("Pitch attitude during velocity control");

    nexttile;
    plot(t_plot, imu_yaw_plot, "LineWidth", 1.1); hold on; grid on;
    plot(t_plot, ekf_yaw_plot, "LineWidth", 1.1);
    xlabel("Time [s]");
    ylabel("Yaw [deg]");
    legend("EBIMU fw", "EKF", "Location", "best");
    title("Yaw attitude during velocity control, unwrapped");

%% ===================== Plot 6: Desired roll/pitch vs actual attitude =====================
if (has_imu_quat || has_imu_fw_rp) && (has_ekf_eul || has_ekf_quat)

    % ctrl start/end 구간만 plot
    idx_plot = idx_start:idx_end;

    t_plot = t(idx_plot);

    roll_des_plot      = roll_des(idx_plot);
    imu_roll_plot      = imu_roll_deg(idx_plot);
    ekf_roll_plot      = ekf_roll_att_deg(idx_plot);

    pitch_des_plot     = pitch_des(idx_plot);
    imu_pitch_plot     = imu_pitch_deg(idx_plot);
    ekf_pitch_plot     = ekf_pitch_att_deg(idx_plot);

    figure("Name", "Desired roll pitch vs actual attitudes - control interval only");
    tiledlayout(2,1, "TileSpacing", "compact");

    nexttile;
    plot(t_plot, roll_des_plot, "--", "LineWidth", 1.2); hold on; grid on;
    plot(t_plot, imu_roll_plot, "LineWidth", 1.1);
    plot(t_plot, ekf_roll_plot, "LineWidth", 1.1);
    ylabel("Roll [deg]");
    legend("roll desired", "EBIMU roll", "EKF roll", "Location", "best");
    title("Roll command tracking during velocity control");

    nexttile;
    plot(t_plot, pitch_des_plot, "--", "LineWidth", 1.2); hold on; grid on;
    plot(t_plot, imu_pitch_plot, "LineWidth", 1.1);
    plot(t_plot, ekf_pitch_plot, "LineWidth", 1.1);
    xlabel("Time [s]");
    ylabel("Pitch [deg]");
    legend("pitch desired", "EBIMU pitch", "EKF pitch", "Location", "best");
    title("Pitch command tracking during velocity control");
end

%% ===================== Local function =====================
function print_metric(name, x, unit)
    x = x(:);
    x = x(isfinite(x));

    if isempty(x)
        fprintf("%-22s : no valid data\n", name);
        return;
    end

    rms_v  = sqrt(mean(x.^2));
    mean_v = mean(x);
    std_v  = std(x);
    p95_v  = prctile(abs(x), 95);
    max_v  = max(abs(x));

    fprintf("%-22s RMS=%8.4f  MEAN=%8.4f  STD=%8.4f  P95(abs)=%8.4f  MAX(abs)=%8.4f  [%s]\n", ...
        name, rms_v, mean_v, std_v, p95_v, max_v, unit);
end

function y = unwrap_deg(x)
    x = x(:);
    y = rad2deg(unwrap(deg2rad(x)));
end

function [roll_deg, pitch_deg, yaw_deg] = quat_wxyz_to_euler_deg(qw, qx, qy, qz)
    qw = qw(:); qx = qx(:); qy = qy(:); qz = qz(:);

    n = sqrt(qw.^2 + qx.^2 + qy.^2 + qz.^2);
    n(n == 0) = 1;
    qw = qw ./ n; qx = qx ./ n; qy = qy ./ n; qz = qz ./ n;

    % ZYX convention: yaw-pitch-roll
    sinr_cosp = 2 .* (qw .* qx + qy .* qz);
    cosr_cosp = 1 - 2 .* (qx.^2 + qy.^2);
    roll = atan2(sinr_cosp, cosr_cosp);

    sinp = 2 .* (qw .* qy - qz .* qx);
    sinp = max(min(sinp, 1), -1);
    pitch = asin(sinp);

    siny_cosp = 2 .* (qw .* qz + qx .* qy);
    cosy_cosp = 1 - 2 .* (qy.^2 + qz.^2);
    yaw = atan2(siny_cosp, cosy_cosp);

    roll_deg  = rad2deg(roll);
    pitch_deg = rad2deg(pitch);
    yaw_deg   = rad2deg(yaw);
end
