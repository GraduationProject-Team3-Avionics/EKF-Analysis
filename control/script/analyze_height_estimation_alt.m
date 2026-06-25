%% STRIX FC baro-INS height estimation and AltHold failure separation
% This script evaluates whether baro-INS height estimation was sane during an
% AltHold test, then separates estimator failure from controller/thrust failure.
%
% Key assumptions:
% - GNSS altitude and GNSS vertical accuracy are intentionally ignored.
% - EKF vertical state is NED:
%       height_up_m = -ekf_pos_d
%       vel_up_mps  = -ekf_vel_d
% - Barometer relative altitude is height-up.
%
% Main questions:
%   1) Did the baro-INS estimator correctly observe height motion?
%   2) Was AltHold target latched and stable?
%   3) Did the controller command more PWM while the vehicle still descended?

clear; clc; close all;

%% User settings
csv_file = "";
if strlength(csv_file) == 0
    csv_file = string(getenv("STRIX_HEIGHT_CSV"));
end
if strlength(csv_file) == 0
    csv_file = "alt_04.CSV";
end
if ~isfile(csv_file)
    fallback = fullfile(pwd, ".cache", "alt_04.CSV");
    if isfile(fallback)
        csv_file = string(fallback);
    else
        [f, p] = uigetfile("*.CSV;*.csv", "Select STRIX height log");
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
descent_detect_vel_threshold_mps = -0.20;
active_height_drop_warn_m = 0.30;
active_height_drop_fail_m = 0.80;
setpoint_span_limit_m = 0.08;
yaw_setpoint_span_limit_deg = 5.0;
cmd_increase_min_us = 10.0;
height_err_clamp_near_m = 0.98;
corr_limit_expected_us = 60.0; % Keep in sync with ALT_CTRL_CORR_LIMIT_US for recent tests.

save_figures = false;
fig_dir = "height_estimation_figures";

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

%% Extract
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
prop_h = optional_col(T, "ekf_prop_height_m");
prop_v_up = -optional_col(T, "ekf_prop_vel_d");
has_prop = any(isfinite(prop_h));

% Optional horizontal velocity-control fields.
vel_active = optional_flag(T, "vel_ctrl_active");
vel_n = optional_col(T, "vel_filt_n_mps");
vel_e = optional_col(T, "vel_filt_e_mps");
vel_err_n = optional_col(T, "vel_err_n_mps");
vel_err_e = optional_col(T, "vel_err_e_mps");

% Optional attitude fields.
roll_deg = optional_col(T, "roll_deg");
pitch_deg = optional_col(T, "pitch_deg");
yaw_deg = optional_col(T, "yaw_deg");

roll_des_deg = optional_col(T, "roll_des_slew_deg");
pitch_des_deg = optional_col(T, "pitch_des_slew_deg");

% Yaw setpoint column name may differ by logger version.
yaw_des_deg = optional_col(T, "yaw_des_slew_deg");
if ~any(isfinite(yaw_des_deg))
    yaw_des_deg = optional_col(T, "yaw_des_deg");
end
if ~any(isfinite(yaw_des_deg))
    yaw_des_deg = optional_col(T, "yaw_sp_deg");
end

has_vel_ctrl = any(isfinite(vel_n)) || any(isfinite(vel_e)) || any(vel_active);
has_attitude = any(isfinite(roll_deg)) || any(isfinite(pitch_deg)) || any(isfinite(yaw_deg));

% Horizontal position is not present in current AltHold logs. If richer logs
% later add it, Figure 5 will use it. Otherwise Figure 5 is skipped.
pos_n = optional_col(T, "ekf_pos_n");
pos_e = optional_col(T, "ekf_pos_e");
if ~any(isfinite(pos_n))
    pos_n = optional_col(T, "pos_n_m");
end
if ~any(isfinite(pos_e))
    pos_e = optional_col(T, "pos_e_m");
end
has_pos = any(isfinite(pos_n)) && any(isfinite(pos_e));

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

segments = get_segments(t, m_judge);
height_diff = ekf_h - baro_h;
alt_meas_diff = alt_meas - ekf_h;
vel_diff = logged_vel_up - ekf_v_up;

%% Report
fprintf("\n=== STRIX Height Estimation / AltHold Separation Check ===\n");
fprintf("File        : %s\n", csv_file);
fprintf("Rows        : %d\n", height(T));
fprintf("Duration    : %.2f s\n", t(end) - t(1));
fprintf("Median rate : %.2f Hz\n", fs_med);
fprintf("Judge mask  : %s\n", judge_name);
fprintf("Judge samples/duration: %d / %.2f s\n", nnz(m_judge), total_duration(segments));
fprintf("Note        : GNSS altitude is ignored.\n");

fprintf("\n--- Basic estimator availability ---\n");
fprintf("baro_ready ratio        : %6.2f %%\n", 100 * mean(baro_ready));
fprintf("ekf_ready ratio         : %6.2f %%\n", 100 * mean(ekf_ready));
fprintf("baro update used ratio  : %6.2f %%\n", 100 * mean(baro_update_used(ekf_ready & baro_ready)));
fprintf("baro rejected ratio     : %6.2f %%\n", 100 * mean(baro_rejected(ekf_ready & baro_ready)));
fprintf("alt_ctrl_active duration: %.2f s\n", total_duration(get_segments(t, alt_active)));

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
end

fprintf("\n--- Control context, for separating estimator vs controller failure ---\n");
if any(m_active)
    print_stats("alt err", alt_err(m_active), "m");
    print_stats("vel err", vel_err(m_active), "m/s");
    print_stats("base PWM", base_pwm(m_active), "us");
    print_stats("cmd PWM", cmd_pwm(m_active), "us");
    print_stats("base latched", base_latched(m_active), "us");
    print_stats("corr", corr_us(m_active), "us");
    fprintf("alt_sp span during active       : %.4f m\n", max(alt_sp(m_active), [], "omitnan") - min(alt_sp(m_active), [], "omitnan"));
    fprintf("corr saturation active ratio    : %.2f %%\n", 100 * mean(corr_sat(m_active)));
    fprintf("corr near expected limit ratio  : %.2f %%  (|corr| > %.1f us)\n", ...
        100 * mean(abs(corr_us(m_active)) > 0.95 * corr_limit_expected_us), 0.95 * corr_limit_expected_us);
    fprintf("height err near clamp ratio     : %.2f %%  (|err| >= %.2f m)\n", ...
        100 * mean(abs(alt_err(m_active)) >= height_err_clamp_near_m), height_err_clamp_near_m);
    fprintf("cmd > base ratio                : %.2f %%\n", 100 * mean(cmd_pwm(m_active) > base_pwm(m_active)));
    print_stats("motor mean", motor_mean(m_active), "us");
end

fprintf("\n--- Attitude / yaw hold context ---\n");
if has_attitude
    print_stats("roll", roll_deg(m_judge), "deg");
    print_stats("pitch", pitch_deg(m_judge), "deg");
    print_stats("yaw", yaw_deg(m_judge), "deg");

    if any(isfinite(roll_des_deg))
        print_stats("roll sp", roll_des_deg(m_judge), "deg");
    end
    if any(isfinite(pitch_des_deg))
        print_stats("pitch sp", pitch_des_deg(m_judge), "deg");
    end
    if any(isfinite(yaw_des_deg))
        print_stats("yaw sp", yaw_des_deg(m_judge), "deg");
        if any(m_active)
            yaw_sp_active_span = angle_span_deg(yaw_des_deg(m_active));
            fprintf("yaw sp span during AltHold active: %.3f deg\n", yaw_sp_active_span);
        end
    else
        fprintf("yaw sp: not logged\n");
    end
else
    fprintf("attitude fields: not logged\n");
end

fprintf("\n--- Pass / warn checks for estimation only ---\n");
check_ge("judge duration", total_duration(segments), min_judge_duration_s, "s");
check_ge("baro update ratio", mean(baro_update_used(m_judge)), baro_update_ratio_min, "");
check_le("baro reject ratio", mean(baro_rejected(m_judge)), baro_reject_ratio_limit, "");
check_le("baro age P95", pct(baro_age_ms(m_judge), 95), baro_age_p95_limit_ms, "ms");
check_le("innovation P95 abs", pct(abs(baro_innov(m_judge)), 95), innov_p95_limit_m, "m");
check_le("EKF-baro P95 abs", pct(abs(height_diff(m_judge)), 95), height_diff_p95_limit_m, "m");
check_ge("height correlation", corr_omitnan(ekf_h(m_judge), baro_h(m_judge)), height_corr_min, "");

if any(m_active)
    active_height_delta = last_valid(ekf_h(m_active)) - first_valid(ekf_h(m_active));
    active_baro_delta = last_valid(baro_h(m_active)) - first_valid(baro_h(m_active));
    active_cmd_delta = last_valid(cmd_pwm(m_active)) - first_valid(cmd_pwm(m_active));
    active_sp_span = max(alt_sp(m_active), [], "omitnan") - min(alt_sp(m_active), [], "omitnan");
    descent_ratio = mean(ekf_v_up(m_active) < descent_detect_vel_threshold_mps);

    check_le("alt_sp active span", active_sp_span, setpoint_span_limit_m, "m");

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

    if mean(corr_sat(m_active)) > 0.20
        fprintf("[CTRL] correction saturated often: correction limit is active.\n");
    elseif max(abs(corr_us(m_active)), [], "omitnan") < 0.75 * corr_limit_expected_us
        fprintf("[CTRL] correction did not reach %.0f us limit. If it still fell, gains/base thrust may be insufficient.\n", corr_limit_expected_us);
    else
        fprintf("[CTRL] correction used much of the allowed range.\n");
    end
end

fprintf("\nInterpretation:\n");
fprintf("- If innovation and EKF-baro difference are small, height estimation is probably OK.\n");
fprintf("- If EKF height and baro height move together while AltHold falls, the failure is likely control authority/PWM cap, not estimation.\n");
fprintf("- If velocity is negative while altitude error is positive, the estimator/controller knows it is falling.\n");
fprintf("- If cmd PWM rises while EKF/baro height still falls, increase authority/base thrust or retune control; do not blame GNSS.\n");
fprintf("- If yaw_sp active span is large during AltHold, yaw latch/hold logic is changing the heading target.\n");
fprintf("- GNSS columns, if present, are intentionally unused.\n");

%% Plots
if save_figures && ~isfolder(fig_dir)
    mkdir(fig_dir);
end

fig1 = figure("Name", "Height estimator overview", "Color", "w");
tiledlayout(5, 1, "TileSpacing", "compact");

nexttile;
plot(t, pressure_pa, "k");
shade_mask(t, m_active);
grid on; ylabel("Pa");
title("Pressure. Falling pressure means rising altitude.");

nexttile;
plot(t, baro_h, "c", "LineWidth", 1.1); hold on;
plot(t, ekf_h, "b", "LineWidth", 1.1);
plot(t, alt_sp, "r");
if has_prop
    plot(t, prop_h, "Color", [0.5 0.5 0.5]);
    legend("baro rel lpf", "EKF height", "alt sp", "prop-only height", "Location", "best");
else
    legend("baro rel lpf", "EKF height", "alt sp", "Location", "best");
end
shade_mask(t, m_active);
grid on; ylabel("height [m]");
title("Baro relative altitude vs EKF height-up");

nexttile;
plot(t, height_diff, "m"); hold on;
yline(0.8, "k--");
yline(-0.8, "k--");
shade_mask(t, m_active);
grid on; ylabel("m");
title("EKF height - baro height");

nexttile;
plot(t, ekf_v_up, "b"); hold on;
plot(t, logged_vel_up, "k--");
yline(0, "r--");
if has_prop
    plot(t, prop_v_up, "Color", [0.5 0.5 0.5]);
    legend("EKF v up", "logged v up", "zero", "prop-only v up", "Location", "best");
else
    legend("EKF v up", "logged v up", "zero", "Location", "best");
end
shade_mask(t, m_active);
grid on; ylabel("m/s");
title("Vertical velocity. Negative means falling.");

nexttile;
stairs(t, ekf_ready, "k"); hold on;
stairs(t, baro_update_used, "g");
stairs(t, baro_rejected, "r");
stairs(t, alt_active, "b");
plot(t, baro_age_ms / max(max(baro_age_ms), 1), "Color", [0.4 0.4 0.9]);
grid on; xlabel("time [s]"); ylim([-0.1 1.2]);
legend("ekf ready", "baro update", "baro reject", "AltHold active", "age norm", "Location", "best");

% fig2 = figure("Name", "Fusion consistency and control context", "Color", "w");
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
% title("Controller sees error. Positive alt/vel err means it wants more thrust.");
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
% plot(t, corr_us, "b");
% shade_mask(t, m_active);
% grid on; xlabel("time [s]"); ylabel("us");
% title("Throttle correction after limits/cap");

% fig3 = figure("Name", "Height estimator scatter", "Color", "w");
% tiledlayout(2, 2, "TileSpacing", "compact");
% 
% nexttile;
% scatter(baro_h(m_judge), ekf_h(m_judge), 20, t(m_judge), "filled");
% grid on; xlabel("baro height [m]"); ylabel("EKF height [m]");
% title("EKF height should follow baro height"); colorbar;
% 
% nexttile;
% scatter(ekf_v_up(m_judge), diff_same_length(ekf_h, t, m_judge), 20, t(m_judge), "filled");
% grid on; xlabel("EKF v up [m/s]"); ylabel("d(EKF height)/dt [m/s]");
% title("Velocity consistency"); colorbar;
% 
% nexttile;
% scatter(alt_err(m_active), cmd_pwm(m_active), 25, t(m_active), "filled");
% grid on; xlabel("alt err [m]"); ylabel("cmd PWM [us]");
% title("During active: error vs command"); colorbar;
% 
% nexttile;
% scatter(vel_err(m_active), cmd_pwm(m_active), 25, t(m_active), "filled");
% grid on; xlabel("vel err [m/s]"); ylabel("cmd PWM [us]");
% title("During active: velocity error vs command"); colorbar;

fig4 = figure("Name", "Horizontal velocity-control context", "Color", "w");
tiledlayout(5, 1, "TileSpacing", "compact");

nexttile;
plot(t, vel_n, "b", "LineWidth", 1.1); hold on;
plot(t, vel_e, "r", "LineWidth", 1.1);
yline(0, "k--");
shade_mask(t, vel_active);
grid on; ylabel("m/s");
legend("v_N filt", "v_E filt", "zero", "Location", "best");
title("Horizontal filtered velocity. Shaded area = velocity control active.");

nexttile;
plot(t, hypot(vel_n, vel_e), "k", "LineWidth", 1.1); hold on;
yline(0.20, "r--");
yline(0.50, "m--");
shade_mask(t, vel_active);
grid on; ylabel("|v_{NE}| [m/s]");
legend("speed XY", "0.20", "0.50", "Location", "best");
title("Horizontal speed magnitude");

nexttile;
plot(t, vel_err_n, "b"); hold on;
plot(t, vel_err_e, "r");
yline(0, "k--");
shade_mask(t, vel_active);
grid on; ylabel("m/s");
legend("v err N", "v err E", "zero", "Location", "best");
title("Velocity-control error. For zero-speed hold, this should oppose measured velocity.");

nexttile;
plot(t, roll_deg, "b"); hold on;
plot(t, roll_des_deg, "b--");
plot(t, pitch_deg, "r");
plot(t, pitch_des_deg, "r--");
yline(0, "k--");
shade_mask(t, vel_active);
grid on; ylabel("deg");
legend("roll", "roll des", "pitch", "pitch des", "zero", "Location", "best");
title("Velocity loop attitude request vs actual attitude");

nexttile;
stairs(t, vel_active, "b"); hold on;
stairs(t, alt_active, "g");
plot(t, motor_mean / max(max(motor_mean), 1), "m");
shade_mask(t, vel_active);
grid on; xlabel("time [s]"); ylim([-0.1 1.2]);
legend("vel ctrl active", "AltHold active", "motor mean norm", "Location", "best");
title("Control-mode context");

if has_pos
    fig5 = figure("Name", "Horizontal position context", "Color", "w");
    tiledlayout(2, 1, "TileSpacing", "compact");

    nexttile;
    plot(t, pos_n, "b", "LineWidth", 1.1); hold on;
    plot(t, pos_e, "r", "LineWidth", 1.1);
    shade_mask(t, vel_active);
    grid on; ylabel("m");
    legend("EKF pos N", "EKF pos E", "Location", "best");
    title("Horizontal EKF position. Position control is not active in this log.");

    nexttile;
    plot(pos_e, pos_n, "k", "LineWidth", 1.1); hold on;
    scatter(first_valid(pos_e), first_valid(pos_n), 35, "g", "filled");
    scatter(last_valid(pos_e), last_valid(pos_n), 35, "r", "filled");
    axis equal; grid on; xlabel("East [m]"); ylabel("North [m]");
    legend("path", "start", "end", "Location", "best");
    title("Horizontal path view. Diagnostic only; no position-control command is logged.");
end

if has_attitude
    fig6 = figure("Name", "Attitude setpoint tracking", "Color", "w");
    tiledlayout(4, 1, "TileSpacing", "compact");

    nexttile;
    plot(t, roll_deg, "b", "LineWidth", 1.1); hold on;
    plot(t, roll_des_deg, "b--", "LineWidth", 1.1);
    yline(0, "k--");
    shade_mask(t, vel_active);
    grid on; ylabel("deg");
    legend("roll", "roll sp", "zero", "Location", "best");
    title("Roll attitude tracking. Shaded area = velocity control active.");

    nexttile;
    plot(t, pitch_deg, "r", "LineWidth", 1.1); hold on;
    plot(t, pitch_des_deg, "r--", "LineWidth", 1.1);
    yline(0, "k--");
    shade_mask(t, vel_active);
    grid on; ylabel("deg");
    legend("pitch", "pitch sp", "zero", "Location", "best");
    title("Pitch attitude tracking. Shaded area = velocity control active.");

    nexttile;
    plot(t, roll_des_deg - roll_deg, "b", "LineWidth", 1.1); hold on;
    plot(t, pitch_des_deg - pitch_deg, "r", "LineWidth", 1.1);
    yline(0, "k--");
    shade_mask(t, vel_active);
    grid on; ylabel("deg");
    legend("roll sp - roll", "pitch sp - pitch", "zero", "Location", "best");
    title("Attitude tracking error. Shaded area = velocity control active.");

    nexttile;
    plot(t, yaw_deg, "k", "LineWidth", 1.1); hold on;
    if any(isfinite(yaw_des_deg))
        plot(t, yaw_des_deg, "k--", "LineWidth", 1.1);
        legend("yaw", "yaw sp", "Location", "best");
    else
        legend("yaw", "Location", "best");
    end
    shade_mask(t, m_active);
    grid on; xlabel("time [s]"); ylabel("deg");
    title("Yaw hold context. Shaded area = AltHold active.");
end

if save_figures
    saveas(fig1, fullfile(fig_dir, "01_height_estimator_overview.png"));
    saveas(fig2, fullfile(fig_dir, "02_fusion_control_context.png"));
    saveas(fig3, fullfile(fig_dir, "03_scatter.png"));
    saveas(fig4, fullfile(fig_dir, "04_velocity_control_context.png"));
    if has_pos
        saveas(fig5, fullfile(fig_dir, "05_position_context.png"));
    end
    if has_attitude
        saveas(fig6, fullfile(fig_dir, "06_attitude_setpoint_tracking.png"));
    end
end

%% Local functions
function assert_has_vars(T, names)
missing = names(~ismember(names, string(T.Properties.VariableNames)));
if ~isempty(missing)
    error("Missing required columns:\n%s", strjoin(missing, newline));
end
end

function x = col(T, name)
x = T.(name);
if ~isnumeric(x)
    x = str2double(string(x));
end
x = double(x);
end

function x = optional_col(T, name)
if ismember(name, string(T.Properties.VariableNames))
    x = col(T, name);
else
    x = nan(height(T), 1);
end
end

function y = flag(T, name)
y = col(T, name) ~= 0;
end

function y = optional_flag(T, name)
y = optional_col(T, name) ~= 0;
end

function p = pct(x, q)
x = x(isfinite(x));
if isempty(x)
    p = nan;
else
    p = prctile(x, q);
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

function y = diff_same_length(x, t, mask)
dxdt = nan(size(x));
good = isfinite(x) & isfinite(t);
idx = find(good);
if numel(idx) >= 2
    dxdt(idx(2:end)) = diff(x(idx)) ./ diff(t(idx));
    dxdt(idx(1)) = dxdt(idx(2));
end
y = dxdt(mask);
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
yl = ylim;
seg = get_segments(t, mask);
for i = 1:size(seg, 1)
    patch([seg(i,1) seg(i,2) seg(i,2) seg(i,1)], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.85 0.95 0.85], "EdgeColor", "none", "FaceAlpha", 0.25, ...
        "HandleVisibility", "off");
end
ylim(yl);
uistack(findobj(gca, "Type", "patch"), "bottom");
end