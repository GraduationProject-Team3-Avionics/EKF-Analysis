%% STRIX FC baro-INS height estimation / AltHold / PosHold separation check
% This script evaluates:
%   1) Whether baro-INS height estimation was sane during AltHold.
%   2) Whether AltHold target was latched and the controller reacted to descent.
%   3) Whether PosHold N/E target was latched and horizontal position was held.
%
% Key assumptions:
% - GNSS altitude is intentionally ignored.
% - EKF vertical state is NED:
%       height_up_m = -ekf_pos_d
%       vel_up_mps  = -ekf_vel_d
% - Barometer relative altitude is height-up.
% - EKF horizontal position is NED:
%       ekf_pos_n, ekf_pos_e are position in meters.
%
% PosHold logging compatibility:
% - If dedicated PosHold columns exist, this script uses them.
% - If they do not exist, it still plots EKF N/E position and velocity as
%   diagnostic context, but it will not claim PosHold latch pass/fail.

clear; clc; close all;

%% User settings
csv_file = "";
if strlength(csv_file) == 0
    csv_file = string(getenv("STRIX_HEIGHT_CSV"));
end
if strlength(csv_file) == 0
    csv_file = string(getenv("STRIX_POSHOLD_CSV"));
end
if strlength(csv_file) == 0
    csv_file = "DATA_059.CSV";
end
if ~isfile(csv_file)
    fallbacks = [
        fullfile(pwd, ".cache", "DATA_059.CSV")
        fullfile(pwd, ".cache", "alt_04.CSV")
        fullfile(pwd, "DATA_059.CSV")
        fullfile(pwd, "alt_04.CSV")
    ];
    hit = "";
    for i = 1:numel(fallbacks)
        if isfile(fallbacks(i))
            hit = string(fallbacks(i));
            break;
        end
    end
    if strlength(hit) > 0
        csv_file = hit;
    else
        [f, p] = uigetfile("*.CSV;*.csv", "Select STRIX log");
        if isequal(f, 0)
            error("No CSV file selected.");
        end
        csv_file = string(fullfile(p, f));
    end
end

% Thresholds for estimator-only judgment.
min_judge_duration_s = 2.0;
pressure_min_pa = 30000;
pressure_max_pa = 125000;
baro_age_p95_limit_ms = 80;
baro_reject_ratio_limit = 0.02;
baro_update_ratio_min = 0.95;
innov_p95_limit_m = 0.80;
height_diff_p95_limit_m = 0.80;
height_corr_min = 0.85;

% AltHold thresholds.
descent_detect_vel_threshold_mps = -0.20;
active_height_drop_warn_m = 0.30;
active_height_drop_fail_m = 0.80;
alt_setpoint_span_limit_m = 0.08;
cmd_increase_min_us = 10.0;
height_err_clamp_near_m = 0.98;
corr_limit_expected_us = 60.0; % Keep in sync with ALT_CTRL_CORR_LIMIT_US.

% PosHold thresholds.
pos_setpoint_span_limit_m = 0.20;
pos_drift_warn_m = 0.50;
pos_drift_fail_m = 1.20;
pos_err_p95_warn_m = 0.80;
pos_speed_rms_warn_mps = 0.25;
pos_speed_p95_warn_mps = 0.50;
pos_err_near_clamp_m = 0.95; % If your PosHold clamps position error at 1 m.
yaw_setpoint_span_limit_deg = 5.0;

save_figures = false;
fig_dir = "height_poshold_figures";

%% Load
opts = detectImportOptions(csv_file);
opts.VariableNamingRule = "preserve";
T = readtable(csv_file, opts);

required = ["timestamp_ms", ...
    "baro_temp_c", "baro_pressure_pa", "baro_rel_alt_lpf_m", "baro_age_ms", ...
    "baro_ready", "ekf_ready", "ekf_pos_d", "ekf_vel_d", ...
    "baro_innov_m", "baro_test_ratio", "baro_update_used", "baro_rejected", ...
    "alt_ctrl_active", "alt_sp_m", "alt_meas_m", "vel_up_mps", ...
    "alt_err_m", "vel_err_mps", "alt_base_pwm_us", "alt_cmd_pwm_us", ...
    "alt_base_pwm_latched", "alt_throttle_corr_us", "alt_throttle_corr_sat", ...
    "M1", "M2", "M3", "M4"];
assert_has_vars(T, required);

t = col(T, "timestamp_ms") * 1e-3;
t = t - t(1);
dt = diff(t);
fs_med = 1 / median(dt(dt > 0), "omitnan");

%% Extract: common / height
baro_temp_c = col(T, "baro_temp_c");
pressure_pa = col(T, "baro_pressure_pa");
baro_h = col(T, "baro_rel_alt_lpf_m");
baro_age_ms = col(T, "baro_age_ms");
baro_ready = flag(T, "baro_ready");

ekf_ready = flag(T, "ekf_ready");
ekf_pos_d = col(T, "ekf_pos_d");
ekf_vel_d = col(T, "ekf_vel_d");
ekf_h = -ekf_pos_d;
ekf_v_up = -ekf_vel_d;

baro_innov = col(T, "baro_innov_m");
baro_test_ratio = col(T, "baro_test_ratio");
baro_update_used = flag(T, "baro_update_used");
baro_rejected = flag(T, "baro_rejected");

alt_active = flag(T, "alt_ctrl_active");
alt_sp = col(T, "alt_sp_m");
alt_meas = col(T, "alt_meas_m");
logged_vel_up = col(T, "vel_up_mps");
alt_err = col(T, "alt_err_m");
vel_err = col(T, "vel_err_mps");
base_pwm = col(T, "alt_base_pwm_us");
cmd_pwm = col(T, "alt_cmd_pwm_us");
base_latched = col(T, "alt_base_pwm_latched");
corr_us = col(T, "alt_throttle_corr_us");
corr_sat = flag(T, "alt_throttle_corr_sat");

M = [col(T, "M1"), col(T, "M2"), col(T, "M3"), col(T, "M4")];
motor_mean = mean(M, 2, "omitnan");

% Optional fields from richer logs.
prop_h = optional_col_any(T, ["ekf_prop_height_m", "prop_height_m"]);
prop_v_up = -optional_col_any(T, ["ekf_prop_vel_d", "prop_vel_d"]);
has_prop = any(isfinite(prop_h));

flight_mode = optional_col_any(T, ["flight_mode"]);

%% Extract: horizontal EKF / velocity-control context
pos_n = optional_col_any(T, ["ekf_pos_n", "pos_n_m", "local_pos_n_m"]);
pos_e = optional_col_any(T, ["ekf_pos_e", "pos_e_m", "local_pos_e_m"]);
has_pos = any(isfinite(pos_n)) && any(isfinite(pos_e));

vel_active = optional_flag_any(T, ["vel_ctrl_active", "vel_active"]);
vel_n = optional_col_any(T, ["vel_filt_n_mps", "vel_n_mps", "ekf_vel_n", "ekf_vel_n_mps"]);
vel_e = optional_col_any(T, ["vel_filt_e_mps", "vel_e_mps", "ekf_vel_e", "ekf_vel_e_mps"]);
vel_err_n = optional_col_any(T, ["vel_err_n_mps", "vel_n_err_mps"]);
vel_err_e = optional_col_any(T, ["vel_err_e_mps", "vel_e_err_mps"]);
vel_sp_n = optional_col_any(T, ["vel_sp_n_mps", "vel_cmd_n_mps", "pos_vel_sp_n_mps", "pos_cmd_vel_n_mps"]);
vel_sp_e = optional_col_any(T, ["vel_sp_e_mps", "vel_cmd_e_mps", "pos_vel_sp_e_mps", "pos_cmd_vel_e_mps"]);

if ~any(isfinite(vel_err_n)) && any(isfinite(vel_sp_n)) && any(isfinite(vel_n))
    vel_err_n = vel_sp_n - vel_n;
end
if ~any(isfinite(vel_err_e)) && any(isfinite(vel_sp_e)) && any(isfinite(vel_e))
    vel_err_e = vel_sp_e - vel_e;
end

has_vel_ctrl = any(isfinite(vel_n)) || any(isfinite(vel_e)) || any(vel_active);

%% Extract: attitude / yaw context
roll_deg = optional_col_any(T, ["roll_deg"]);
pitch_deg = optional_col_any(T, ["pitch_deg"]);
yaw_deg = optional_col_any(T, ["yaw_deg", "imu_yaw_deg"]);

roll_des_deg = optional_col_any(T, ["roll_des_slew_deg", "roll_des_deg", "roll_sp_deg"]);
pitch_des_deg = optional_col_any(T, ["pitch_des_slew_deg", "pitch_des_deg", "pitch_sp_deg"]);
yaw_des_deg = optional_col_any(T, ["yaw_des_slew_deg", "yaw_des_deg", "yaw_sp_deg"]);

has_attitude = any(isfinite(roll_deg)) || any(isfinite(pitch_deg)) || any(isfinite(yaw_deg));

%% Extract: PosHold, with broad logger-name compatibility
pos_enabled = optional_flag_any(T, ["pos_ctrl_enabled", "pos_hold_enabled", "poshold_enabled"]);
pos_allowed = optional_flag_any(T, ["pos_ctrl_allowed", "pos_hold_allowed", "poshold_allowed"]);
pos_active_raw = optional_flag_any(T, ["pos_ctrl_active", "pos_hold_active", "poshold_active"]);
pos_latched = optional_flag_any(T, ["pos_hold_latched", "pos_ctrl_latched", "poshold_latched"]);

pos_sp_n = optional_col_any(T, ["pos_sp_n_m", "pos_hold_sp_n_m", "pos_target_n_m", "hold_pos_n_m", "target_pos_n_m"]);
pos_sp_e = optional_col_any(T, ["pos_sp_e_m", "pos_hold_sp_e_m", "pos_target_e_m", "hold_pos_e_m", "target_pos_e_m"]);
pos_meas_n = optional_col_any(T, ["pos_meas_n_m", "pos_ctrl_meas_n_m", "pos_hold_meas_n_m"]);
pos_meas_e = optional_col_any(T, ["pos_meas_e_m", "pos_ctrl_meas_e_m", "pos_hold_meas_e_m"]);

if ~any(isfinite(pos_meas_n)) && has_pos
    pos_meas_n = pos_n;
end
if ~any(isfinite(pos_meas_e)) && has_pos
    pos_meas_e = pos_e;
end

pos_err_n = optional_col_any(T, ["pos_err_n_m", "pos_hold_err_n_m", "pos_ctrl_err_n_m"]);
pos_err_e = optional_col_any(T, ["pos_err_e_m", "pos_hold_err_e_m", "pos_ctrl_err_e_m"]);
if ~any(isfinite(pos_err_n)) && any(isfinite(pos_sp_n)) && any(isfinite(pos_meas_n))
    pos_err_n = pos_sp_n - pos_meas_n;
end
if ~any(isfinite(pos_err_e)) && any(isfinite(pos_sp_e)) && any(isfinite(pos_meas_e))
    pos_err_e = pos_sp_e - pos_meas_e;
end

pos_p_term_n = optional_col_any(T, ["pos_p_term_n_mps", "pos_p_n_mps", "pos_ctrl_p_n_mps"]);
pos_p_term_e = optional_col_any(T, ["pos_p_term_e_mps", "pos_p_e_mps", "pos_ctrl_p_e_mps"]);
pos_cmd_vel_n = optional_col_any(T, ["pos_cmd_vel_n_mps", "pos_vel_cmd_n_mps", "pos_vel_sp_n_mps"]);
pos_cmd_vel_e = optional_col_any(T, ["pos_cmd_vel_e_mps", "pos_vel_cmd_e_mps", "pos_vel_sp_e_mps"]);
pos_cmd_vel_sat = optional_flag_any(T, ["pos_cmd_vel_sat", "pos_vel_cmd_sat", "pos_ctrl_vel_sat"]);

has_poshold_active_flag = has_any_var(T, ["pos_ctrl_active", "pos_hold_active", "poshold_active"]);
has_poshold_latched_flag = has_any_var(T, ["pos_hold_latched", "pos_ctrl_latched", "poshold_latched"]);
if has_poshold_active_flag
    pos_active = pos_active_raw;
elseif has_poshold_latched_flag
    pos_active = pos_latched;
else
    pos_active = false(height(T), 1);
end
has_poshold_flag = has_poshold_active_flag || has_poshold_latched_flag;
has_poshold_sp = any(isfinite(pos_sp_n)) && any(isfinite(pos_sp_e));
has_poshold_err = any(isfinite(pos_err_n)) && any(isfinite(pos_err_e));
has_poshold = has_poshold_flag || has_poshold_sp || has_poshold_err;

%% Masks
m_est = ekf_ready & baro_ready & baro_update_used & ~baro_rejected;
m_active = m_est & alt_active;
if any(m_active)
    m_judge = m_active;
    judge_name = "AltHold active + EKF/baro valid";
else
    m_judge = m_est;
    judge_name = "EKF/baro valid";
end

m_pos_valid = ekf_ready & isfinite(pos_meas_n) & isfinite(pos_meas_e);
m_pos_active = m_pos_valid & pos_active;
if any(m_pos_active)
    m_pos_judge = m_pos_active;
    if has_poshold_active_flag
        pos_judge_name = "PosHold active + EKF horizontal valid";
    elseif has_poshold_latched_flag
        pos_judge_name = "PosHold latched + EKF horizontal valid";
    else
        pos_judge_name = "PosHold judge-active + EKF horizontal valid";
    end
elseif has_pos
    m_pos_judge = m_pos_valid;
    if has_poshold_latched_flag
        pos_judge_name = "EKF horizontal valid, PosHold latched is always false";
    else
        pos_judge_name = "EKF horizontal valid, PosHold active/latched flag not present";
    end
else
    m_pos_judge = false(size(t));
    pos_judge_name = "No horizontal position columns";
end

segments = get_segments(t, m_judge);
pos_segments = get_segments(t, m_pos_judge);
height_diff = ekf_h - baro_h;
alt_meas_diff = alt_meas - ekf_h;
vel_diff = logged_vel_up - ekf_v_up;

speed_xy = hypot(vel_n, vel_e);
pos_err_mag = hypot(pos_err_n, pos_err_e);

if has_pos
    pos_n0 = first_valid(pos_meas_n(m_pos_judge));
    pos_e0 = first_valid(pos_meas_e(m_pos_judge));
    pos_drift_from_start = hypot(pos_meas_n - pos_n0, pos_meas_e - pos_e0);
else
    pos_drift_from_start = nan(size(t));
end

%% Report
fprintf("\n=== STRIX Height / AltHold / PosHold Separation Check ===\n");
fprintf("File        : %s\n", char(csv_file));
fprintf("Rows        : %d\n", height(T));
fprintf("Duration    : %.2f s\n", t(end) - t(1));
fprintf("Median rate : %.2f Hz\n", fs_med);
fprintf("Height judge: %s\n", judge_name);
fprintf("Height judge samples/duration: %d / %.2f s\n", nnz(m_judge), total_duration(segments));
fprintf("Pos judge   : %s\n", pos_judge_name);
fprintf("Pos judge samples/duration   : %d / %.2f s\n", nnz(m_pos_judge), total_duration(pos_segments));
fprintf("Note        : GNSS altitude is ignored.\n");

fprintf("\n--- Basic estimator availability ---\n");
fprintf("baro_ready ratio        : %6.2f %%\n", 100 * mean(baro_ready));
fprintf("ekf_ready ratio         : %6.2f %%\n", 100 * mean(ekf_ready));
fprintf("baro update used ratio  : %6.2f %%\n", 100 * mean_or_nan(baro_update_used(ekf_ready & baro_ready)));
fprintf("baro rejected ratio     : %6.2f %%\n", 100 * mean_or_nan(baro_rejected(ekf_ready & baro_ready)));
fprintf("alt_ctrl_active duration: %.2f s\n", total_duration(get_segments(t, alt_active)));
if has_poshold_active_flag
    fprintf("pos_ctrl_active duration: %.2f s\n", total_duration(get_segments(t, pos_active)));
elseif has_poshold_latched_flag
    fprintf("pos_hold_latched duration: %.2f s  (used as PosHold judge mask)\n", total_duration(get_segments(t, pos_active)));
else
    fprintf("pos_ctrl_active duration: not logged\n");
end
if any(isfinite(flight_mode))
    print_stats("flight mode", flight_mode, "");
end

fprintf("\n--- Pressure / timing sanity ---\n");
print_stats("pressure", pressure_pa(m_judge), "Pa");
print_stats("temperature", baro_temp_c(m_judge), "degC");
print_stats("baro age", baro_age_ms(m_judge), "ms");
if median(pressure_pa(m_judge), "omitnan") < pressure_min_pa || median(pressure_pa(m_judge), "omitnan") > pressure_max_pa
    fprintf("WARNING: pressure median is outside BMP390L physical range. Check logging units.\n");
end

fprintf("\n--- Height and velocity, height-up convention ---\n");
print_stats("baro height", baro_h(m_judge), "m");
print_stats("EKF height", ekf_h(m_judge), "m");
print_stats("EKF-baro", height_diff(m_judge), "m");
print_stats("EKF v_up", ekf_v_up(m_judge), "m/s");
print_stats("logged v_up", logged_vel_up(m_judge), "m/s");
print_stats("logged-EKF v", vel_diff(m_judge), "m/s");
if has_prop
    print_stats("prop height", prop_h(m_judge), "m");
    print_stats("prop v_up", prop_v_up(m_judge), "m/s");
end

fprintf("\n--- Baro fusion consistency ---\n");
print_stats("innovation", baro_innov(m_judge), "m");
print_stats("test ratio", baro_test_ratio(m_judge), "");
fprintf("innovation P95 abs / max abs: %.3f / %.3f m\n", pct(abs(baro_innov(m_judge)), 95), max(abs(baro_innov(m_judge)), [], "omitnan"));
fprintf("EKF-baro P95 abs / max abs : %.3f / %.3f m\n", pct(abs(height_diff(m_judge)), 95), max(abs(height_diff(m_judge)), [], "omitnan"));
fprintf("corr(EKF height, baro)     : %.3f\n", corr_omitnan(ekf_h(m_judge), baro_h(m_judge)));

fprintf("\n--- Did estimator see the descent? ---\n");
if any(m_active)
    m_desc = m_active & ekf_v_up < descent_detect_vel_threshold_mps;
    active_h0 = first_valid(ekf_h(m_active));
    active_h1 = last_valid(ekf_h(m_active));
    active_b0 = first_valid(baro_h(m_active));
    active_b1 = last_valid(baro_h(m_active));
    active_sp0 = first_valid(alt_sp(m_active));
    active_sp1 = last_valid(alt_sp(m_active));
    active_cmd0 = first_valid(cmd_pwm(m_active));
    active_cmd1 = last_valid(cmd_pwm(m_active));
    fprintf("active descent samples      : %d / %d\n", nnz(m_desc), nnz(m_active));
    fprintf("active mean vel_up          : %.3f m/s\n", mean(ekf_v_up(m_active), "omitnan"));
    fprintf("active min vel_up           : %.3f m/s\n", min(ekf_v_up(m_active), [], "omitnan"));
    fprintf("active start/end height     : %.3f -> %.3f m, delta %+.3f m\n", active_h0, active_h1, active_h1 - active_h0);
    fprintf("active start/end baro       : %.3f -> %.3f m, delta %+.3f m\n", active_b0, active_b1, active_b1 - active_b0);
    fprintf("active start/end alt_sp     : %.3f -> %.3f m, delta %+.3f m\n", active_sp0, active_sp1, active_sp1 - active_sp0);
    fprintf("active start/end cmd PWM    : %.1f -> %.1f us, delta %+.1f us\n", active_cmd0, active_cmd1, active_cmd1 - active_cmd0);
else
    fprintf("AltHold active samples: none\n");
end

fprintf("\n--- AltHold control context ---\n");
if any(m_active)
    print_stats("alt err", alt_err(m_active), "m");
    print_stats("vel err", vel_err(m_active), "m/s");
    print_stats("base PWM", base_pwm(m_active), "us");
    print_stats("cmd PWM", cmd_pwm(m_active), "us");
    print_stats("base latched", base_latched(m_active), "us");
    print_stats("corr", corr_us(m_active), "us");
    fprintf("alt_sp span during active       : %.4f m\n", span_omitnan(alt_sp(m_active)));
    fprintf("corr saturation active ratio    : %.2f %%\n", 100 * mean_or_nan(corr_sat(m_active)));
    fprintf("corr near expected limit ratio  : %.2f %%  (|corr| > %.1f us)\n", ...
        100 * mean_or_nan(abs(corr_us(m_active)) > 0.95 * corr_limit_expected_us), 0.95 * corr_limit_expected_us);
    fprintf("height err near clamp ratio     : %.2f %%  (|err| >= %.2f m)\n", ...
        100 * mean_or_nan(abs(alt_err(m_active)) >= height_err_clamp_near_m), height_err_clamp_near_m);
    fprintf("cmd > base ratio                : %.2f %%\n", 100 * mean_or_nan(cmd_pwm(m_active) > base_pwm(m_active)));
    print_stats("motor mean", motor_mean(m_active), "us");
else
    fprintf("AltHold control fields exist, but active samples are none.\n");
end

fprintf("\n--- PosHold horizontal position context ---\n");
if has_pos
    print_stats("pos N", pos_meas_n(m_pos_judge), "m");
    print_stats("pos E", pos_meas_e(m_pos_judge), "m");
    print_stats("drift start", pos_drift_from_start(m_pos_judge), "m");
    print_stats("vel N", vel_n(m_pos_judge), "m/s");
    print_stats("vel E", vel_e(m_pos_judge), "m/s");
    print_stats("speed XY", speed_xy(m_pos_judge), "m/s");
else
    fprintf("Horizontal EKF position columns are not logged.\n");
end

if has_poshold
    if has_poshold_flag
        if has_any_var(T, ["pos_ctrl_enabled", "pos_hold_enabled", "poshold_enabled"])
            fprintf("pos enabled ratio              : %.2f %%\n", 100 * mean_or_nan(pos_enabled));
        else
            fprintf("pos enabled ratio              : not logged\n");
        end
        if has_any_var(T, ["pos_ctrl_allowed", "pos_hold_allowed", "poshold_allowed"])
            fprintf("pos allowed ratio              : %.2f %%\n", 100 * mean_or_nan(pos_allowed));
        else
            fprintf("pos allowed ratio              : not logged\n");
        end
        fprintf("pos judge-active ratio         : %.2f %%\n", 100 * mean_or_nan(pos_active));
        if has_poshold_latched_flag
            fprintf("pos latched ratio              : %.2f %%\n", 100 * mean_or_nan(pos_latched));
        else
            fprintf("pos latched ratio              : not logged\n");
        end
    end
    if has_poshold_sp
        print_stats("pos sp N", pos_sp_n(m_pos_judge), "m");
        print_stats("pos sp E", pos_sp_e(m_pos_judge), "m");
        fprintf("pos_sp N span during judge      : %.4f m\n", span_omitnan(pos_sp_n(m_pos_judge)));
        fprintf("pos_sp E span during judge      : %.4f m\n", span_omitnan(pos_sp_e(m_pos_judge)));
        fprintf("pos_sp NE span during judge     : %.4f m\n", path_span_ne(pos_sp_n(m_pos_judge), pos_sp_e(m_pos_judge)));
    else
        fprintf("pos setpoint columns: not logged\n");
    end
    if has_poshold_err
        print_stats("pos err N", pos_err_n(m_pos_judge), "m");
        print_stats("pos err E", pos_err_e(m_pos_judge), "m");
        print_stats("pos err XY", pos_err_mag(m_pos_judge), "m");
        fprintf("pos err near clamp ratio        : %.2f %%  (|errXY| >= %.2f m)\n", ...
            100 * mean_or_nan(pos_err_mag(m_pos_judge) >= pos_err_near_clamp_m), pos_err_near_clamp_m);
    else
        fprintf("pos error columns: not logged and cannot be reconstructed without pos_sp.\n");
    end
    if any(isfinite(pos_cmd_vel_n)) || any(isfinite(pos_cmd_vel_e))
        print_stats("pos cmd vN", pos_cmd_vel_n(m_pos_judge), "m/s");
        print_stats("pos cmd vE", pos_cmd_vel_e(m_pos_judge), "m/s");
        print_stats("pos cmd speed", hypot(pos_cmd_vel_n(m_pos_judge), pos_cmd_vel_e(m_pos_judge)), "m/s");
        fprintf("pos velocity command saturation : %.2f %%\n", 100 * mean_or_nan(pos_cmd_vel_sat(m_pos_judge)));
    end
else
    fprintf("Dedicated PosHold columns are not logged. Position plot is diagnostic only.\n");
end

fprintf("\n--- Velocity / attitude bridge for PosHold separation ---\n");
if has_vel_ctrl
    print_stats("vel err N", vel_err_n(m_pos_judge), "m/s");
    print_stats("vel err E", vel_err_e(m_pos_judge), "m/s");
    print_stats("vel err XY", hypot(vel_err_n(m_pos_judge), vel_err_e(m_pos_judge)), "m/s");
else
    fprintf("Velocity-control fields are not logged.\n");
end
if has_attitude
    print_stats("roll", roll_deg(m_pos_judge), "deg");
    print_stats("pitch", pitch_deg(m_pos_judge), "deg");
    print_stats("yaw", yaw_deg(m_pos_judge), "deg");
    if any(isfinite(roll_des_deg))
        print_stats("roll sp", roll_des_deg(m_pos_judge), "deg");
        print_stats("roll err", roll_des_deg(m_pos_judge) - roll_deg(m_pos_judge), "deg");
    end
    if any(isfinite(pitch_des_deg))
        print_stats("pitch sp", pitch_des_deg(m_pos_judge), "deg");
        print_stats("pitch err", pitch_des_deg(m_pos_judge) - pitch_deg(m_pos_judge), "deg");
    end
    if any(isfinite(yaw_des_deg))
        print_stats("yaw sp", yaw_des_deg(m_pos_judge), "deg");
        if any(m_active)
            fprintf("yaw sp span during AltHold active: %.3f deg\n", angle_span_deg(yaw_des_deg(m_active)));
        end
        if any(m_pos_active)
            fprintf("yaw sp span during PosHold active: %.3f deg\n", angle_span_deg(yaw_des_deg(m_pos_active)));
        end
    else
        fprintf("yaw sp: not logged\n");
    end
else
    fprintf("attitude fields: not logged\n");
end

fprintf("\n--- Pass / warn checks for height estimation only ---\n");
check_ge("judge duration", total_duration(segments), min_judge_duration_s, "s");
check_ge("baro update ratio", mean_or_nan(baro_update_used(m_judge)), baro_update_ratio_min, "");
check_le("baro reject ratio", mean_or_nan(baro_rejected(m_judge)), baro_reject_ratio_limit, "");
check_le("baro age P95", pct(baro_age_ms(m_judge), 95), baro_age_p95_limit_ms, "ms");
check_le("innovation P95 abs", pct(abs(baro_innov(m_judge)), 95), innov_p95_limit_m, "m");
check_le("EKF-baro P95 abs", pct(abs(height_diff(m_judge)), 95), height_diff_p95_limit_m, "m");
check_ge("height correlation", corr_omitnan(ekf_h(m_judge), baro_h(m_judge)), height_corr_min, "");

fprintf("\n--- Pass / warn checks for AltHold behavior ---\n");
if any(m_active)
    active_height_delta = last_valid(ekf_h(m_active)) - first_valid(ekf_h(m_active));
    active_baro_delta = last_valid(baro_h(m_active)) - first_valid(baro_h(m_active));
    active_cmd_delta = last_valid(cmd_pwm(m_active)) - first_valid(cmd_pwm(m_active));
    active_sp_span = span_omitnan(alt_sp(m_active));
    descent_ratio = mean_or_nan(ekf_v_up(m_active) < descent_detect_vel_threshold_mps);

    check_le("alt_sp active span", active_sp_span, alt_setpoint_span_limit_m, "m");

    if any(isfinite(yaw_des_deg))
        yaw_sp_active_span = angle_span_deg(yaw_des_deg(m_active));
        check_le("yaw_sp active span", yaw_sp_active_span, yaw_setpoint_span_limit_deg, "deg");
    end

    if active_height_delta < -active_height_drop_fail_m
        fprintf("[FAIL] active height drop: EKF height dropped %.3f m during AltHold active.\n", -active_height_delta);
    elseif active_height_delta < -active_height_drop_warn_m
        fprintf("[WARN] active height drop: EKF height dropped %.3f m during AltHold active.\n", -active_height_delta);
    else
        fprintf("[PASS] active height drop: EKF height delta %.3f m is small.\n", active_height_delta);
    end

    if active_height_delta < -active_height_drop_warn_m && active_baro_delta < -active_height_drop_warn_m
        fprintf("[PASS] estimator motion agreement: EKF and baro both report descent.\n");
    elseif descent_ratio > 0.20
        fprintf("[PASS] estimator motion agreement: EKF velocity reports descent in %.1f%% of active samples.\n", 100 * descent_ratio);
    else
        fprintf("[WARN] estimator motion agreement: descent is not clear from EKF velocity/baro trend.\n");
    end

    if active_height_delta < -active_height_drop_warn_m && active_cmd_delta > cmd_increase_min_us
        fprintf("[CTRL] PWM increased by %.1f us while height dropped %.3f m: likely thrust/authority/tuning issue, not estimator blindness.\n", ...
            active_cmd_delta, -active_height_delta);
    end

    if mean_or_nan(corr_sat(m_active)) > 0.20
        fprintf("[CTRL] correction saturated often: correction limit is active.\n");
    elseif max(abs(corr_us(m_active)), [], "omitnan") < 0.75 * corr_limit_expected_us
        fprintf("[CTRL] correction did not reach %.0f us limit. If it still fell, gains/base thrust may be insufficient.\n", corr_limit_expected_us);
    else
        fprintf("[CTRL] correction used much of the allowed range.\n");
    end
else
    fprintf("[N/A] AltHold was not active.\n");
end

fprintf("\n--- Pass / warn checks for PosHold behavior ---\n");
if any(m_pos_judge)
    pos_duration = total_duration(pos_segments);
    pos_drift_final = last_valid(pos_drift_from_start(m_pos_judge));
    pos_drift_max = max(pos_drift_from_start(m_pos_judge), [], "omitnan");
    speed_rms = rms_omitnan(speed_xy(m_pos_judge));
    speed_p95 = pct(speed_xy(m_pos_judge), 95);

    check_ge("pos judge duration", pos_duration, min_judge_duration_s, "s");
    check_le("XY speed RMS", speed_rms, pos_speed_rms_warn_mps, "m/s");
    check_le("XY speed P95", speed_p95, pos_speed_p95_warn_mps, "m/s");

    if has_poshold_sp
        pos_sp_span = path_span_ne(pos_sp_n(m_pos_judge), pos_sp_e(m_pos_judge));
        check_le("pos_sp NE span", pos_sp_span, pos_setpoint_span_limit_m, "m");
    else
        fprintf("[N/A] pos_sp NE span: position setpoint is not logged.\n");
    end

    if has_poshold_err
        check_le("pos err XY P95", pct(pos_err_mag(m_pos_judge), 95), pos_err_p95_warn_m, "m");
    else
        fprintf("[N/A] pos err XY P95: position error is not logged/reconstructable.\n");
    end

    if has_poshold_flag && any(m_pos_active)
        if pos_drift_final > pos_drift_fail_m
            fprintf("[FAIL] PosHold drift: final drift from active start is %.3f m, max %.3f m.\n", pos_drift_final, pos_drift_max);
        elseif pos_drift_final > pos_drift_warn_m
            fprintf("[WARN] PosHold drift: final drift from active start is %.3f m, max %.3f m.\n", pos_drift_final, pos_drift_max);
        else
            fprintf("[PASS] PosHold drift: final drift %.3f m, max %.3f m.\n", pos_drift_final, pos_drift_max);
        end
    elseif has_pos
        fprintf("[DIAG] PosHold active flag is missing/false. EKF horizontal drift from judge start: final %.3f m, max %.3f m.\n", ...
            pos_drift_final, pos_drift_max);
    end

    if has_poshold_sp && has_poshold_err
        pos_err_start = first_valid(pos_err_mag(m_pos_judge));
        pos_err_end = last_valid(pos_err_mag(m_pos_judge));
        fprintf("[POS] pos error XY start/end: %.3f -> %.3f m, delta %+.3f m\n", ...
            pos_err_start, pos_err_end, pos_err_end - pos_err_start);
        if span_omitnan(pos_sp_n(m_pos_judge)) <= pos_setpoint_span_limit_m && ...
                span_omitnan(pos_sp_e(m_pos_judge)) <= pos_setpoint_span_limit_m && ...
                pos_err_end > pos_err_p95_warn_m
            fprintf("[POS] Setpoint is stable but error is large: inspect velocity loop, attitude tracking, thrust margin, and FC mounting bias.\n");
        end
    end

    if any(isfinite(roll_des_deg)) && any(isfinite(pitch_des_deg))
        roll_track_p95 = pct(abs(roll_des_deg(m_pos_judge) - roll_deg(m_pos_judge)), 95);
        pitch_track_p95 = pct(abs(pitch_des_deg(m_pos_judge) - pitch_deg(m_pos_judge)), 95);
        fprintf("[POS] attitude tracking P95 abs: roll %.3f deg, pitch %.3f deg\n", roll_track_p95, pitch_track_p95);
    end
else
    fprintf("[N/A] No horizontal position samples available for PosHold judgment.\n");
end

fprintf("\nInterpretation:\n");
fprintf("- If innovation and EKF-baro difference are small, height estimation is probably OK.\n");
fprintf("- If EKF height and baro height move together while AltHold falls, the failure is likely control authority/PWM cap, not estimation.\n");
fprintf("- If PosHold setpoint span is small but N/E error or drift grows, latch logic is probably OK; inspect velocity loop, attitude tracking, thrust margin, and FC mounting bias.\n");
fprintf("- If PosHold setpoint span is large during active hold, first check command burst handling and setpoint latch/reset logic.\n");
fprintf("- If horizontal speed stays high while position error is large, PosHold is asking for correction but the velocity/attitude/thrust chain is not arresting motion.\n");
fprintf("- GNSS columns, if present, are intentionally unused for altitude judgment.\n");

%% Plots
if save_figures && ~isfolder(fig_dir)
    mkdir(fig_dir);
end

figs = gobjects(0);
fig_names = strings(0);

fig1 = figure("Name", "Height estimator overview", "Color", "w");
figs(end+1) = fig1;
fig_names(end+1) = "01_height_estimator_overview.png";

tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

% 1) Baro / EKF height / AltHold setpoint
nexttile;
plot(t, baro_h, "c", "LineWidth", 1.1); hold on;
plot(t, ekf_h, "b", "LineWidth", 1.1);
plot(t, alt_sp, "r", "LineWidth", 1.1);

shade_mask(t, m_active);
grid on;
ylabel("height [m]");
title("Baro relative altitude vs EKF height-up");
legend("baro rel lpf", "EKF height", "alt sp", "Location", "best");

% 2) Vertical velocity
nexttile;
plot(t, ekf_v_up, "b", "LineWidth", 1.1); hold on;
plot(t, logged_vel_up, "k--", "LineWidth", 1.0);
yline(0, "r--");

shade_mask(t, m_active);
grid on;
xlabel("time [s]");
ylabel("m/s");
title("Vertical velocity. Negative means falling.");
legend("EKF v up", "logged v up", "zero", "Location", "best");
% fig2 = figure("Name", "AltHold fusion and control context", "Color", "w");
% figs(end+1) = fig2; fig_names(end+1) = "02_althold_fusion_control_context.png";
% tiledlayout(4, 1, "TileSpacing", "compact");
% 
% nexttile;
% plot(t, baro_innov, "m"); hold on;
% plot(t, baro_test_ratio, "k");
% shade_mask(t, m_active);
% grid on; ylabel("m / ratio");
% legend("baro innovation", "test ratio", "Location", "best");
% title("Baro fusion innovation");
% 
% nexttile;
% plot(t, alt_err, "r"); hold on;
% plot(t, vel_err, "b");
% yline(0, "k--");
% shade_mask(t, m_active);
% grid on; ylabel("err");
% legend("alt err [m]", "vel err [m/s]", "Location", "best");
% title("Controller sees vertical error. Positive alt/vel err means it wants more thrust.");
% 
% nexttile;
% plot(t, base_pwm, "k"); hold on;
% plot(t, cmd_pwm, "b");
% plot(t, base_latched, "Color", [0.5 0.5 0.5]);
% plot(t, motor_mean, "m");
% shade_mask(t, m_active);
% grid on; ylabel("PWM [us]");
% legend("base", "cmd", "base latched", "motor mean", "Location", "best");
% title("PWM context. If cmd is capped while falling, controller authority is insufficient.");
% 
% nexttile;
% plot(t, corr_us, "b"); hold on;
% yline(corr_limit_expected_us, "k--");
% yline(-corr_limit_expected_us, "k--");
% shade_mask(t, m_active);
% grid on; xlabel("time [s]"); ylabel("us");
% title("Throttle correction after limits/cap");

if has_pos
    % Plot only the original shaded PosHold-active interval.
    m_fig3 = m_pos_active;

    if ~any(m_fig3)
        warning("Figure 3: m_pos_active has no samples. Falling back to m_pos_judge.");
        m_fig3 = m_pos_judge;
    end

    pos_n_plot = pos_meas_n;
    pos_e_plot = pos_meas_e;
    pos_n_plot(~m_fig3) = nan;
    pos_e_plot(~m_fig3) = nan;

    if has_poshold_sp
        pos_sp_n_plot = pos_sp_n;
        pos_sp_e_plot = pos_sp_e;
        pos_sp_n_plot(~m_fig3) = nan;
        pos_sp_e_plot(~m_fig3) = nan;
    end

    t_start = first_valid(t(m_fig3));
    t_end = last_valid(t(m_fig3));

    fig3 = figure("Name", "PosHold position context", "Color", "w");

    figs(end+1) = fig3;
    fig_names(end+1) = "03_poshold_position_context.png";

    tiledlayout(3, 1, ...
        "TileSpacing", "compact", ...
        "Padding", "compact");

    % 1) N/E position during PosHold active only
    nexttile;
    plot(t, pos_n_plot, "b", "LineWidth", 1.1); hold on;
    plot(t, pos_e_plot, "r", "LineWidth", 1.1);

    if has_poshold_sp
        plot(t, pos_sp_n_plot, "b--", "LineWidth", 1.1);
        plot(t, pos_sp_e_plot, "r--", "LineWidth", 1.1);
        legend("pos N", "pos E", "sp N", "sp E", "Location", "best");
    else
        legend("pos N", "pos E", "Location", "best");
    end

    if isfinite(t_start) && isfinite(t_end) && t_end > t_start
        xlim([t_start t_end]);
    end

    grid on;
    ylabel("m");
    title("Horizontal position during PosHold active");

    % 2) Horizontal path view during PosHold active only
    nexttile([2 1]);
    plot(pos_e_plot, pos_n_plot, "k", "LineWidth", 1.2); hold on;

    scatter(first_valid(pos_e_plot), ...
            first_valid(pos_n_plot), ...
            45, "g", "filled");

    scatter(last_valid(pos_e_plot), ...
            last_valid(pos_n_plot), ...
            45, "r", "filled");

    if has_poshold_sp
        scatter(first_valid(pos_sp_e_plot), ...
                first_valid(pos_sp_n_plot), ...
                55, "b", "filled");
    end

    axis equal;
    grid on;
    xlabel("East [m]");
    ylabel("North [m]");

    if has_poshold_sp
        legend("path", "start", "end", "sp", "Location", "best");
    else
        legend("path", "start", "end", "Location", "best");
    end

    title("Horizontal path view during PosHold active");
end

if has_vel_ctrl
    % Plot only the original shaded Figure 4 interval.
    % Original shade mask was: vel_active | m_pos_active
    m_fig4 = vel_active | m_pos_active;

    if ~any(m_fig4)
        warning("Figure 4: vel_active | m_pos_active has no samples. Falling back to m_pos_judge.");
        m_fig4 = m_pos_judge;
    end

    vel_n_plot = vel_n;
    vel_e_plot = vel_e;
    vel_n_plot(~m_fig4) = nan;
    vel_e_plot(~m_fig4) = nan;

    if any(isfinite(vel_sp_n)) || any(isfinite(vel_sp_e))
        vel_sp_n_plot = vel_sp_n;
        vel_sp_e_plot = vel_sp_e;
        vel_sp_n_plot(~m_fig4) = nan;
        vel_sp_e_plot(~m_fig4) = nan;
    end

    t_start = first_valid(t(m_fig4));
    t_end = last_valid(t(m_fig4));

    fig4 = figure("Name", "Velocity and attitude bridge", "Color", "w");
    figs(end+1) = fig4;
    fig_names(end+1) = "04_velocity_attitude_bridge.png";

    plot(t, vel_n_plot, "b", "LineWidth", 1.1); hold on;
    plot(t, vel_e_plot, "r", "LineWidth", 1.1);

    if any(isfinite(vel_sp_n)) || any(isfinite(vel_sp_e))
        plot(t, vel_sp_n_plot, "b--", "LineWidth", 1.1);
        plot(t, vel_sp_e_plot, "r--", "LineWidth", 1.1);
        legend("v_N", "v_E", "v sp N", "v sp E", "Location", "best");
    else
        legend("v_N", "v_E", "Location", "best");
    end

    yline(0, "k--", "HandleVisibility", "off");

    if isfinite(t_start) && isfinite(t_end) && t_end > t_start
        xlim([t_start t_end]);
    end

    grid on;
    xlabel("time [s]");
    ylabel("m/s");
    title("Horizontal velocity and velocity setpoint");
end

if has_attitude
    % Plot only the original shaded attitude/yaw context interval.
    % For PosHold analysis, this mainly means PosHold active or velocity-control active.
    m_fig5 = m_pos_active | vel_active;

    if ~any(m_fig5)
        warning("Figure 5: m_pos_active | vel_active has no samples. Falling back to m_active | m_pos_active.");
        m_fig5 = m_active | m_pos_active;
    end

    % roll_plot = roll_deg;
    % pitch_plot = pitch_deg;
    % yaw_plot = yaw_deg;
        % Mounting/visual correction for actual attitude only
    pitch_actual_offset_deg = -5.0;
    yaw_actual_offset_deg   = +2.0;

    roll_plot = roll_deg;
    pitch_plot = pitch_deg + pitch_actual_offset_deg;
    yaw_plot = mod((yaw_deg + yaw_actual_offset_deg) + 180, 360) - 180;

    roll_plot(~m_fig5) = nan;
    pitch_plot(~m_fig5) = nan;
    yaw_plot(~m_fig5) = nan;

    if any(isfinite(roll_des_deg))
        roll_des_plot = roll_des_deg;
        roll_des_plot(~m_fig5) = nan;
    end

    if any(isfinite(pitch_des_deg))
        pitch_des_plot = pitch_des_deg;
        pitch_des_plot(~m_fig5) = nan;
    end

    if any(isfinite(yaw_des_deg))
        yaw_des_plot = yaw_des_deg;
        yaw_des_plot(~m_fig5) = nan;
    end

    t_start = first_valid(t(m_fig5));
    t_end = last_valid(t(m_fig5));

    fig5 = figure("Name", "Attitude and yaw hold context", "Color", "w");
    figs(end+1) = fig5;
    fig_names(end+1) = "05_attitude_yaw_context.png";

    tiledlayout(3, 1, ...
        "TileSpacing", "compact", ...
        "Padding", "compact");

    % 1) Roll
    nexttile;
    plot(t, roll_plot, "b", "LineWidth", 1.1); hold on;
    if any(isfinite(roll_des_deg))
        plot(t, roll_des_plot, "b--", "LineWidth", 1.1);
        legend("roll", "roll sp", "Location", "best");
    else
        legend("roll", "Location", "best");
    end
    yline(0, "k--");

    if isfinite(t_start) && isfinite(t_end) && t_end > t_start
        xlim([t_start t_end]);
    end

    grid on;
    ylabel("deg");
    title("Roll attitude tracking");

    % 2) Pitch
    nexttile;
    plot(t, pitch_plot, "r", "LineWidth", 1.1); hold on;
    if any(isfinite(pitch_des_deg))
        plot(t, pitch_des_plot, "r--", "LineWidth", 1.1);
        legend("pitch", "pitch sp", "Location", "best");
    else
        legend("pitch", "Location", "best");
    end
    yline(0, "k--");

    if isfinite(t_start) && isfinite(t_end) && t_end > t_start
        xlim([t_start t_end]);
    end

    grid on;
    ylabel("deg");
    title("Pitch attitude tracking");

    % 3) Yaw
    nexttile;
    plot(t, yaw_plot, "k", "LineWidth", 1.1); hold on;
    if any(isfinite(yaw_des_deg))
        plot(t, yaw_des_plot, "k--", "LineWidth", 1.1);
        legend("yaw", "yaw sp", "Location", "best");
    else
        legend("yaw", "Location", "best");
    end

    if isfinite(t_start) && isfinite(t_end) && t_end > t_start
        xlim([t_start t_end]);
    end

    grid on;
    xlabel("time [s]");
    ylabel("deg");
    title("Yaw hold context");
end

if save_figures
    for i = 1:numel(figs)
        saveas(figs(i), fullfile(fig_dir, fig_names(i)));
    end
end

%% Final RMSE / max-error summary
fprintf("\n=== Final RMSE / Max Error Summary ===\n");

% 1) Attitude: setpoint - measured attitude
m_att_rmse = isfinite(roll_deg) | isfinite(pitch_deg) | isfinite(yaw_deg);
if any(m_pos_active)
    m_att_rmse = m_att_rmse & m_pos_active;
elseif any(vel_active)
    m_att_rmse = m_att_rmse & vel_active;
end

fprintf("\n--- Attitude Error ---\n");
if any(isfinite(roll_des_deg)) && any(isfinite(roll_deg))
    roll_err = roll_des_deg(m_att_rmse) - roll_deg(m_att_rmse);
    fprintf("roll RMSE       : %.4f deg\n", rmse_omitnan(roll_err));
    fprintf("roll max error  : %.4f deg\n", max_abs_omitnan(roll_err));
else
    fprintf("roll error      : N/A\n");
end

if any(isfinite(pitch_des_deg)) && any(isfinite(pitch_deg))
    pitch_err = pitch_des_deg(m_att_rmse) - pitch_deg(m_att_rmse);
    fprintf("pitch RMSE      : %.4f deg\n", rmse_omitnan(pitch_err));
    fprintf("pitch max error : %.4f deg\n", max_abs_omitnan(pitch_err));
else
    fprintf("pitch error     : N/A\n");
end

if any(isfinite(yaw_des_deg)) && any(isfinite(yaw_deg))
    yaw_err = angle_err_deg(yaw_des_deg(m_att_rmse), yaw_deg(m_att_rmse));
    fprintf("yaw RMSE        : %.4f deg\n", rmse_omitnan(yaw_err));
    fprintf("yaw max error   : %.4f deg\n", max_abs_omitnan(yaw_err));
else
    fprintf("yaw error       : N/A\n");
end

% 2) Position: position setpoint - measured position
fprintf("\n--- Position Error ---\n");
if has_poshold_sp && has_pos
    m_pos_rmse = m_pos_judge;
    if any(m_pos_active)
        m_pos_rmse = m_pos_active;
    end

    pos_err_n_calc = pos_sp_n(m_pos_rmse) - pos_meas_n(m_pos_rmse);
    pos_err_e_calc = pos_sp_e(m_pos_rmse) - pos_meas_e(m_pos_rmse);
    pos_err_xy_calc = hypot(pos_err_n_calc, pos_err_e_calc);

    fprintf("pos N RMSE      : %.4f m\n", rmse_omitnan(pos_err_n_calc));
    fprintf("pos N max error : %.4f m\n", max_abs_omitnan(pos_err_n_calc));
    fprintf("pos E RMSE      : %.4f m\n", rmse_omitnan(pos_err_e_calc));
    fprintf("pos E max error : %.4f m\n", max_abs_omitnan(pos_err_e_calc));
    fprintf("pos XY RMSE     : %.4f m\n", sqrt(mean_omitnan(pos_err_xy_calc.^2)));
    fprintf("pos XY max error: %.4f m\n", max_omitnan(pos_err_xy_calc));
else
    fprintf("position error  : N/A\n");
end

% 3) Altitude: altitude setpoint - EKF height-up
fprintf("\n--- Altitude Error ---\n");
if any(isfinite(alt_sp)) && any(isfinite(ekf_h))
    m_alt_rmse = m_judge;
    if any(m_active)
        m_alt_rmse = m_active;
    end

    alt_err_calc = alt_sp(m_alt_rmse) - ekf_h(m_alt_rmse);

    fprintf("altitude RMSE   : %.4f m\n", rmse_omitnan(alt_err_calc));
    fprintf("altitude max err: %.4f m\n", max_abs_omitnan(alt_err_calc));
else
    fprintf("altitude error  : N/A\n");
end

% 4) Velocity: velocity setpoint - measured horizontal velocity
fprintf("\n--- Velocity Error ---\n");
if any(isfinite(vel_sp_n)) && any(isfinite(vel_sp_e)) && any(isfinite(vel_n)) && any(isfinite(vel_e))
    m_vel_rmse = isfinite(vel_sp_n) & isfinite(vel_sp_e) & isfinite(vel_n) & isfinite(vel_e);
    if any(vel_active | m_pos_active)
        m_vel_rmse = m_vel_rmse & (vel_active | m_pos_active);
    end

    vel_err_n_calc = vel_sp_n(m_vel_rmse) - vel_n(m_vel_rmse);
    vel_err_e_calc = vel_sp_e(m_vel_rmse) - vel_e(m_vel_rmse);
    vel_err_xy_calc = hypot(vel_err_n_calc, vel_err_e_calc);

    fprintf("vel N RMSE      : %.4f m/s\n", rmse_omitnan(vel_err_n_calc));
    fprintf("vel N max error : %.4f m/s\n", max_abs_omitnan(vel_err_n_calc));
    fprintf("vel E RMSE      : %.4f m/s\n", rmse_omitnan(vel_err_e_calc));
    fprintf("vel E max error : %.4f m/s\n", max_abs_omitnan(vel_err_e_calc));
    fprintf("vel XY RMSE     : %.4f m/s\n", sqrt(mean_omitnan(vel_err_xy_calc.^2)));
    fprintf("vel XY max error: %.4f m/s\n", max_omitnan(vel_err_xy_calc));
else
    fprintf("velocity error  : N/A\n");
end

%% Local functions
function assert_has_vars(T, names)
missing = names(~ismember(names, string(T.Properties.VariableNames)));
if ~isempty(missing)
    error("Missing required columns:\n%s", strjoin(missing, newline));
end
end

function tf = has_any_var(T, names)
tf = any(ismember(string(names), string(T.Properties.VariableNames)));
end

function x = col(T, name)
x = T.(char(name));
if ~isnumeric(x)
    x = str2double(string(x));
end
x = double(x);
end

function x = optional_col_any(T, names)
x = nan(height(T), 1);
vars = string(T.Properties.VariableNames);
names = string(names);
for i = 1:numel(names)
    if ismember(names(i), vars)
        x = col(T, names(i));
        return;
    end
end
end

function y = flag(T, name)
y = col(T, name) ~= 0;
end

function y = optional_flag_any(T, names)
x = optional_col_any(T, names);
y = isfinite(x) & x ~= 0;
end

function y = mean_or_nan(x)
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = mean(x);
end
end

function p = pct(x, q)
x = x(isfinite(x));
if isempty(x)
    p = nan;
else
    p = prctile(x, q);
end
end

function y = rms_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = sqrt(mean(x.^2));
end
end

function s = span_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    s = nan;
else
    s = max(x) - min(x);
end
end

function s = path_span_ne(n, e)
good = isfinite(n) & isfinite(e);
if ~any(good)
    s = nan;
else
    n = n(good);
    e = e(good);
    s = hypot(max(n) - min(n), max(e) - min(e));
end
end

function print_stats(name, x, unit)
x = x(isfinite(x));
if isempty(x)
    fprintf("%-14s: no samples\n", name);
    return;
end
fprintf("%-14s mean/std/P05/P50/P95/P95abs/min/max/span = %+8.4f / %7.4f / %+8.4f / %+8.4f / %+8.4f / %7.4f / %+8.4f / %+8.4f / %7.4f %s\n", ...
    name, mean(x), std(x), prctile(x, 5), median(x), prctile(x, 95), ...
    prctile(abs(x), 95), min(x), max(x), max(x)-min(x), unit);
end

function seg = get_segments(t, mask)
mask = mask(:);
rise = find(diff([false; mask]) == 1);
fall = find(diff([mask; false]) == -1);
seg = zeros(numel(rise), 3);
for i = 1:numel(rise)
    seg(i, :) = [t(rise(i)), t(fall(i)), t(fall(i)) - t(rise(i))];
end
end

function d = total_duration(seg)
if isempty(seg)
    d = 0;
else
    d = sum(seg(:, 3));
end
end

function r = corr_omitnan(a, b)
good = isfinite(a) & isfinite(b);
if nnz(good) < 3
    r = nan;
else
    C = corrcoef(a(good), b(good));
    r = C(1,2);
end
end

function check_ge(name, value, limit, unit)
if isnan(value)
    verdict = "N/A";
elseif value >= limit
    verdict = "PASS";
else
    verdict = "WARN";
end
fprintf("[%s] %-24s %8.3f >= %-8.3f %s\n", verdict, name + ":", value, limit, unit);
end

function check_le(name, value, limit, unit)
if isnan(value)
    verdict = "N/A";
elseif value <= limit
    verdict = "PASS";
else
    verdict = "WARN";
end
fprintf("[%s] %-24s %8.3f <= %-8.3f %s\n", verdict, name + ":", value, limit, unit);
end

function v = first_valid(x)
x = x(isfinite(x));
if isempty(x)
    v = nan;
else
    v = x(1);
end
end

function v = last_valid(x)
x = x(isfinite(x));
if isempty(x)
    v = nan;
else
    v = x(end);
end
end

function s = angle_span_deg(x)
x = x(isfinite(x));
if isempty(x)
    s = nan;
    return;
end
xr = deg2rad(x);
c = mean(cos(xr));
sn = mean(sin(xr));
center = atan2(sn, c);
d = atan2(sin(xr - center), cos(xr - center));
s = rad2deg(max(d) - min(d));
end

function shade_mask(t, mask)
if ~any(mask)
    return;
end
yl = ylim;
seg = get_segments(t, mask);
for i = 1:size(seg, 1)
    patch([seg(i,1) seg(i,2) seg(i,2) seg(i,1)], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.85 0.95 0.85], "EdgeColor", "none", "FaceAlpha", 0.25, ...
        "HandleVisibility", "off");
end
ylim(yl);
p = findobj(gca, "Type", "patch");
if ~isempty(p)
    uistack(p, "bottom");
end
end

function y = rmse_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = sqrt(mean(x.^2));
end
end

function y = mean_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = mean(x);
end
end

function e = angle_err_deg(sp_deg, meas_deg)
e = atan2d(sind(sp_deg - meas_deg), cosd(sp_deg - meas_deg));
end

function y = max_abs_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = max(abs(x));
end
end

function y = max_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = max(x);
end
end