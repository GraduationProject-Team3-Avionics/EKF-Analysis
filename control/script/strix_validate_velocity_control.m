clear; clc; close all;

%% STRIX velocity-control validation script
% Purpose:
%   Validate horizontal velocity damping around 0 m/s.
%   Check target velocity, measured velocity, acceleration/tilt command,
%   and actual attitude tracking.
%
% Usage:
%   1) Set csv_file below, or leave it empty and select a CSV from dialog.
%   2) Run this script.
%   3) Read PASS/WARN/FAIL summary and inspect plots.
%
% No toolbox dependency:
%   This script avoids corr() and prctile().

%% User settings
csv_file = "";    % Example: "data/260620_2.CSV"

% Optional time window. Use inf to include until the end.
analysis_start_sec = 0.0;
analysis_end_sec = inf;

% Use only velocity-controller active and valid samples for main judgment.
use_active_valid_only = true;

% Plot/export settings
show_plots = true;
save_plot = false;
plot_file = "";   % empty -> auto name beside CSV

% Thresholds. Adjust after enough real hover data is collected.
limit.min_active_duration_sec = 10.0;
limit.vel_rms_ok_mps = 0.12;
limit.vel_p95_ok_mps = 0.20;
limit.vel_max_warn_mps = 0.35;
limit.speed_under_010_ok_pct = 70.0;

limit.accel_sat_warn_pct = 1.0;
limit.tilt_sat_warn_pct = 1.0;
limit.command_direction_corr_max = -0.85;   % corr(v, acc_cmd) should be negative.

limit.des_tilt_p95_ok_deg = 2.0;
limit.des_tilt_max_warn_deg = 5.0;

% Tether tests can have large fixed attitude offsets. These are warnings,
% not hard velocity-control failures.
limit.att_err_rms_warn_deg = 4.0;
limit.att_err_p95_warn_deg = 7.0;
limit.att_corr_warn = 0.35;
limit.actual_tilt_p95_warn_deg = 8.0;

% Barometer sanity only. This does not judge XY velocity control.
limit.baro_pressure_min_pa = 30000;
limit.baro_pressure_max_pa = 120000;

%% Load CSV
if strlength(string(csv_file)) == 0
    env_file = string(getenv("STRIX_LOG_CSV"));
    if strlength(env_file) > 0
        csv_file = env_file;
    end
end

if strlength(string(csv_file)) == 0 || ~isfile(csv_file)
    [f, p] = uigetfile({"*.CSV;*.csv", "CSV logs (*.CSV, *.csv)"}, ...
        "Select STRIX log CSV");
    if isequal(f, 0)
        error("No CSV file selected.");
    end
    csv_file = string(fullfile(p, f));
end

T = readtable(csv_file, "VariableNamingRule", "preserve");
required = [
    "timestamp_ms"
    "vel_ctrl_active"
    "vel_ctrl_valid"
    "vel_ctrl_accel_saturated"
    "vel_ctrl_tilt_saturated"
    "vel_sp_n_mps"
    "vel_sp_e_mps"
    "vel_meas_n_mps"
    "vel_meas_e_mps"
    "vel_filt_n_mps"
    "vel_filt_e_mps"
    "vel_err_n_mps"
    "vel_err_e_mps"
    "vel_acc_cmd_n_mps2"
    "vel_acc_cmd_e_mps2"
    "roll_des_deg"
    "pitch_des_deg"
    "roll_deg"
    "pitch_deg"
    "ekf_pwm_mean"
];
assert_required_columns(T, required);

t = (T.timestamp_ms - T.timestamp_ms(1)) * 1e-3;
time_mask = t >= analysis_start_sec & t <= analysis_end_sec;

active_valid = logical(T.vel_ctrl_active) & logical(T.vel_ctrl_valid);
if use_active_valid_only
    judge_mask = time_mask & active_valid;
else
    judge_mask = time_mask;
end
% 
% if nnz(judge_mask) < 2
%     error("Not enough samples in selected analysis mask.");
% end

%% Derived signals
v_sp_xy = hypot(T.vel_sp_n_mps, T.vel_sp_e_mps);
v_filt_xy = hypot(T.vel_filt_n_mps, T.vel_filt_e_mps);
v_meas_xy = hypot(T.vel_meas_n_mps, T.vel_meas_e_mps);

if has_column(T, "ekf_vel_n") && has_column(T, "ekf_vel_e")
    v_ekf_xy = hypot(T.ekf_vel_n, T.ekf_vel_e);
else
    v_ekf_xy = nan(height(T), 1);
end

acc_cmd_xy = hypot(T.vel_acc_cmd_n_mps2, T.vel_acc_cmd_e_mps2);
des_tilt = hypot(T.roll_des_deg, T.pitch_des_deg);
actual_tilt = hypot(T.roll_deg, T.pitch_deg);

roll_err = T.roll_deg - T.roll_des_deg;
pitch_err = T.pitch_deg - T.pitch_des_deg;

segments = logical_segments(judge_mask);
active_duration = sum_segment_duration(t, segments);

%% Stats
S.file = string(csv_file);
S.rows = height(T);
S.duration = t(end);
S.median_hz = 1 / finite_median(diff(t));
S.mask_samples = nnz(judge_mask);
S.active_duration = active_duration;
S.active_ratio_pct = 100 * finite_mean(double(active_valid(time_mask)));
S.pwm_mean = finite_mean(T.ekf_pwm_mean(judge_mask));

S.vel_target = mag_stats(v_sp_xy, judge_mask);
S.vel_filt = mag_stats(v_filt_xy, judge_mask);
S.vel_meas = mag_stats(v_meas_xy, judge_mask);
S.vel_ekf = mag_stats(v_ekf_xy, judge_mask);
S.acc_cmd = mag_stats(acc_cmd_xy, judge_mask);
S.des_tilt = mag_stats(des_tilt, judge_mask);
S.actual_tilt = mag_stats(actual_tilt, judge_mask);

S.vel_under_010_pct = 100 * finite_mean(double(v_filt_xy(judge_mask) < 0.10));
S.vel_under_015_pct = 100 * finite_mean(double(v_filt_xy(judge_mask) < 0.15));
S.vel_under_020_pct = 100 * finite_mean(double(v_filt_xy(judge_mask) < 0.20));

S.accel_sat_pct = 100 * finite_mean(double(logical(T.vel_ctrl_accel_saturated(judge_mask))));
S.tilt_sat_pct = 100 * finite_mean(double(logical(T.vel_ctrl_tilt_saturated(judge_mask))));

S.corr_vn_accn = corr_basic(T.vel_filt_n_mps(judge_mask), T.vel_acc_cmd_n_mps2(judge_mask));
S.corr_ve_acce = corr_basic(T.vel_filt_e_mps(judge_mask), T.vel_acc_cmd_e_mps2(judge_mask));
S.corr_roll = corr_basic(T.roll_des_deg(judge_mask), T.roll_deg(judge_mask));
S.corr_pitch = corr_basic(T.pitch_des_deg(judge_mask), T.pitch_deg(judge_mask));

S.roll_err = signed_stats(roll_err, judge_mask);
S.pitch_err = signed_stats(pitch_err, judge_mask);
S.roll_err_demean = demean_error_stats(T.roll_des_deg, T.roll_deg, judge_mask);
S.pitch_err_demean = demean_error_stats(T.pitch_des_deg, T.pitch_deg, judge_mask);

S.roll_des = signed_stats(T.roll_des_deg, judge_mask);
S.pitch_des = signed_stats(T.pitch_des_deg, judge_mask);
S.roll_actual = signed_stats(T.roll_deg, judge_mask);
S.pitch_actual = signed_stats(T.pitch_deg, judge_mask);

%% Print report
fprintf("=== STRIX Velocity-Control Validation ===\n");
fprintf("File        : %s\n", S.file);
fprintf("Rows        : %d\n", S.rows);
fprintf("Duration    : %.2f s\n", S.duration);
fprintf("Median rate : %.2f Hz\n", S.median_hz);
fprintf("Judge mask  : %d samples, %.2f s active duration, PWM mean %.0f\n\n", ...
    S.mask_samples, S.active_duration, S.pwm_mean);

results = {};
results{end+1} = check_result("active duration", ...
    S.active_duration >= limit.min_active_duration_sec, ...
    sprintf("%.2f s >= %.2f s", S.active_duration, limit.min_active_duration_sec), ...
    "Not enough active data.");

results{end+1} = check_result("target velocity zero", ...
    S.vel_target.max <= 1e-6, ...
    sprintf("target |Vxy| max %.4f m/s", S.vel_target.max), ...
    "Target velocity is not zero.");

results{end+1} = check_result("velocity RMS", ...
    S.vel_filt.rms <= limit.vel_rms_ok_mps, ...
    sprintf("filtered |Vxy| RMS %.3f <= %.3f m/s", S.vel_filt.rms, limit.vel_rms_ok_mps), ...
    "Velocity RMS is high.");

results{end+1} = check_result("velocity P95", ...
    S.vel_filt.p95 <= limit.vel_p95_ok_mps, ...
    sprintf("filtered |Vxy| P95 %.3f <= %.3f m/s", S.vel_filt.p95, limit.vel_p95_ok_mps), ...
    "Velocity P95 is high.");

results{end+1} = check_warn("velocity max", ...
    S.vel_filt.max <= limit.vel_max_warn_mps, ...
    sprintf("filtered |Vxy| max %.3f <= %.3f m/s", S.vel_filt.max, limit.vel_max_warn_mps), ...
    "Peak velocity is high.");

results{end+1} = check_result("under 0.10 m/s ratio", ...
    S.vel_under_010_pct >= limit.speed_under_010_ok_pct, ...
    sprintf("%.1f%% >= %.1f%%", S.vel_under_010_pct, limit.speed_under_010_ok_pct), ...
    "Too many samples above 0.10 m/s.");

results{end+1} = check_warn("accel saturation", ...
    S.accel_sat_pct <= limit.accel_sat_warn_pct, ...
    sprintf("%.2f%% <= %.2f%%", S.accel_sat_pct, limit.accel_sat_warn_pct), ...
    "Acceleration command saturates.");

results{end+1} = check_warn("tilt saturation", ...
    S.tilt_sat_pct <= limit.tilt_sat_warn_pct, ...
    sprintf("%.2f%% <= %.2f%%", S.tilt_sat_pct, limit.tilt_sat_warn_pct), ...
    "Tilt command saturates.");

results{end+1} = check_result("N command direction", ...
    S.corr_vn_accn <= limit.command_direction_corr_max, ...
    sprintf("corr(vN, accN) %.3f <= %.3f", S.corr_vn_accn, limit.command_direction_corr_max), ...
    "N-axis acceleration command is not opposite to velocity.");

results{end+1} = check_result("E command direction", ...
    S.corr_ve_acce <= limit.command_direction_corr_max, ...
    sprintf("corr(vE, accE) %.3f <= %.3f", S.corr_ve_acce, limit.command_direction_corr_max), ...
    "E-axis acceleration command is not opposite to velocity.");

results{end+1} = check_result("desired tilt P95", ...
    S.des_tilt.p95 <= limit.des_tilt_p95_ok_deg, ...
    sprintf("desired tilt P95 %.2f <= %.2f deg", S.des_tilt.p95, limit.des_tilt_p95_ok_deg), ...
    "Velocity controller is asking for large tilt.");

results{end+1} = check_warn("desired tilt max", ...
    S.des_tilt.max <= limit.des_tilt_max_warn_deg, ...
    sprintf("desired tilt max %.2f <= %.2f deg", S.des_tilt.max, limit.des_tilt_max_warn_deg), ...
    "Large instantaneous desired tilt.");

results{end+1} = check_warn("actual tilt P95", ...
    S.actual_tilt.p95 <= limit.actual_tilt_p95_warn_deg, ...
    sprintf("actual tilt P95 %.2f <= %.2f deg", S.actual_tilt.p95, limit.actual_tilt_p95_warn_deg), ...
    "Actual attitude is much larger than command/test setup may dominate.");

results{end+1} = check_warn("roll tracking", ...
    S.roll_err.rms <= limit.att_err_rms_warn_deg && S.roll_err.p95_abs <= limit.att_err_p95_warn_deg, ...
    sprintf("roll err RMS/P95abs %.2f / %.2f deg", S.roll_err.rms, S.roll_err.p95_abs), ...
    "Roll does not track desired attitude well.");

results{end+1} = check_warn("pitch tracking", ...
    S.pitch_err.rms <= limit.att_err_rms_warn_deg && S.pitch_err.p95_abs <= limit.att_err_p95_warn_deg, ...
    sprintf("pitch err RMS/P95abs %.2f / %.2f deg", S.pitch_err.rms, S.pitch_err.p95_abs), ...
    "Pitch does not track desired attitude well.");

results{end+1} = check_warn("roll dynamic correlation", ...
    S.corr_roll >= limit.att_corr_warn, ...
    sprintf("corr(roll_des, roll) %.3f >= %.3f", S.corr_roll, limit.att_corr_warn), ...
    "Roll actual motion is weakly related to roll command.");

results{end+1} = check_warn("pitch dynamic correlation", ...
    S.corr_pitch >= limit.att_corr_warn, ...
    sprintf("corr(pitch_des, pitch) %.3f >= %.3f", S.corr_pitch, limit.att_corr_warn), ...
    "Pitch actual motion is weakly related to pitch command.");

if has_column(T, "baro_pressure_pa")
    bp = mag_stats(T.baro_pressure_pa, time_mask);
    results{end+1} = check_warn("baro pressure range", ...
        bp.min >= limit.baro_pressure_min_pa && bp.max <= limit.baro_pressure_max_pa, ...
        sprintf("pressure min/max %.1f / %.1f Pa", bp.min, bp.max), ...
        "Baro pressure logging/scaling looks suspicious.");
end

print_results(results);

fprintf("\n--- Core stats, judge mask ---\n");
print_mag("target |Vxy|", S.vel_target, "m/s");
print_mag("filtered |Vxy|", S.vel_filt, "m/s");
print_mag("measured |Vxy|", S.vel_meas, "m/s");
if any(isfinite(v_ekf_xy(judge_mask)))
    print_mag("EKF |Vxy|", S.vel_ekf, "m/s");
end
print_mag("acc cmd |xy|", S.acc_cmd, "m/s^2");
print_mag("desired tilt", S.des_tilt, "deg");
print_mag("actual tilt", S.actual_tilt, "deg");

fprintf("\n--- Signed attitude stats ---\n");
print_signed("roll_des", S.roll_des, "deg");
print_signed("pitch_des", S.pitch_des, "deg");
print_signed("roll actual", S.roll_actual, "deg");
print_signed("pitch actual", S.pitch_actual, "deg");

fprintf("\n--- Tracking ---\n");
fprintf("roll raw err mean/RMS/P95abs/maxabs     = %+7.3f / %.3f / %.3f / %.3f deg\n", ...
    S.roll_err.mean, S.roll_err.rms, S.roll_err.p95_abs, S.roll_err.max_abs);
fprintf("roll demean err RMS/P95abs/corr         = %.3f / %.3f / %+7.3f\n", ...
    S.roll_err_demean.rms, S.roll_err_demean.p95_abs, S.corr_roll);
fprintf("pitch raw err mean/RMS/P95abs/maxabs    = %+7.3f / %.3f / %.3f / %.3f deg\n", ...
    S.pitch_err.mean, S.pitch_err.rms, S.pitch_err.p95_abs, S.pitch_err.max_abs);
fprintf("pitch demean err RMS/P95abs/corr        = %.3f / %.3f / %+7.3f\n", ...
    S.pitch_err_demean.rms, S.pitch_err_demean.p95_abs, S.corr_pitch);

fprintf("\n--- Direction sanity ---\n");
fprintf("corr(v_filt_n, acc_cmd_n): %+7.3f\n", S.corr_vn_accn);
fprintf("corr(v_filt_e, acc_cmd_e): %+7.3f\n", S.corr_ve_acce);

fprintf("\n--- Velocity threshold ---\n");
fprintf("|Vxy| < 0.10 m/s : %6.2f %%\n", S.vel_under_010_pct);
fprintf("|Vxy| < 0.15 m/s : %6.2f %%\n", S.vel_under_015_pct);
fprintf("|Vxy| < 0.20 m/s : %6.2f %%\n", S.vel_under_020_pct);

fprintf("\n--- Active segments in judge mask ---\n");
for i = 1:size(segments, 1)
    s = segments(i, 1);
    e = segments(i, 2);
    m = false(height(T), 1);
    m(s:e) = true;
    fprintf("seg %d: %.2f ~ %.2f s, dur %.2f s, pwm mean %.0f\n", ...
        i, t(s), t(e), t(e) - t(s), finite_mean(T.ekf_pwm_mean(m)));
    fprintf("  filtered |Vxy| mean/RMS/P95/max = %.3f / %.3f / %.3f / %.3f m/s\n", ...
        finite_mean(v_filt_xy(m)), finite_rms(v_filt_xy(m)), finite_percentile(v_filt_xy(m), 95), finite_max(v_filt_xy(m)));
    fprintf("  desired tilt P95/max = %.2f / %.2f deg\n", ...
        finite_percentile(des_tilt(m), 95), finite_max(des_tilt(m)));
    fprintf("  actual  tilt P95/max = %.2f / %.2f deg\n", ...
        finite_percentile(actual_tilt(m), 95), finite_max(actual_tilt(m)));
    fprintf("  roll err RMS/P95abs = %.2f / %.2f deg\n", ...
        finite_rms(roll_err(m)), finite_percentile(abs(roll_err(m)), 95));
    fprintf("  pitch err RMS/P95abs = %.2f / %.2f deg\n", ...
        finite_rms(pitch_err(m)), finite_percentile(abs(pitch_err(m)), 95));
end

%% Plot
if show_plots
    fig = figure("Name", "STRIX Velocity-Control Validation", "Color", "w");
    try
        fig.Position = [80 60 1450 950];
    catch
    end

    ax1 = subplot(5, 1, 1); hold(ax1, "on");
    plot(ax1, t, T.vel_sp_n_mps, "k--", "LineWidth", 1.0);
    plot(ax1, t, T.vel_sp_e_mps, "--", "Color", [0.35 0.35 0.35], "LineWidth", 1.0);
    plot(ax1, t, T.vel_filt_n_mps, "b", "LineWidth", 1.1);
    plot(ax1, t, T.vel_filt_e_mps, "Color", [0.85 0.33 0.10], "LineWidth", 1.1);
    add_active_background(ax1, t, judge_mask);
    ylabel(ax1, "Vel [m/s]");
    title(ax1, "Target velocity and filtered current velocity");
    legend(ax1, "target N", "target E", "filtered N", "filtered E", "Location", "eastoutside");
    grid(ax1, "on");

    ax2 = subplot(5, 1, 2); hold(ax2, "on");
    plot(ax2, t, v_filt_xy, "Color", [0.49 0.18 0.56], "LineWidth", 1.2);
    plot(ax2, t, v_meas_xy, "Color", [0.93 0.69 0.13], "LineWidth", 0.9);
    plot(ax2, [t(1) t(end)], [0.10 0.10], "g--", "LineWidth", 0.8);
    plot(ax2, [t(1) t(end)], [0.20 0.20], "r--", "LineWidth", 0.8);
    add_active_background(ax2, t, judge_mask);
    ylabel(ax2, "|Vxy| [m/s]");
    title(ax2, "Horizontal speed magnitude");
    legend(ax2, "filtered |Vxy|", "measured |Vxy|", "0.10", "0.20", "Location", "eastoutside");
    grid(ax2, "on");

    ax3 = subplot(5, 1, 3); hold(ax3, "on");
    plot(ax3, t, T.vel_acc_cmd_n_mps2, "b", "LineWidth", 1.0);
    plot(ax3, t, T.vel_acc_cmd_e_mps2, "Color", [0.85 0.33 0.10], "LineWidth", 1.0);
    plot(ax3, t, acc_cmd_xy, "r", "LineWidth", 0.9);
    add_active_background(ax3, t, judge_mask);
    ylabel(ax3, "Acc cmd [m/s^2]");
    title(ax3, "Velocity error acceleration command");
    legend(ax3, "acc cmd N", "acc cmd E", "|acc cmd|", "Location", "eastoutside");
    grid(ax3, "on");

    ax4 = subplot(5, 1, 4); hold(ax4, "on");
    plot(ax4, t, T.roll_des_deg, "b--", "LineWidth", 1.2);
    plot(ax4, t, T.pitch_des_deg, "--", "Color", [0.85 0.33 0.10], "LineWidth", 1.2);
    plot(ax4, t, T.roll_deg, "b", "LineWidth", 0.9);
    plot(ax4, t, T.pitch_deg, "Color", [0.85 0.33 0.10], "LineWidth", 0.9);
    add_active_background(ax4, t, judge_mask);
    ylabel(ax4, "Att [deg]");
    title(ax4, "Desired attitude vs actual attitude");
    legend(ax4, "roll des", "pitch des", "roll actual", "pitch actual", "Location", "eastoutside");
    grid(ax4, "on");

    ax5 = subplot(5, 1, 5); hold(ax5, "on");
    yyaxis(ax5, "left");
    plot(ax5, t, T.ekf_pwm_mean, "Color", [0.25 0.25 0.25], "LineWidth", 1.1);
    ylabel(ax5, "PWM [us]");
    yyaxis(ax5, "right");
    plot(ax5, t, double(active_valid), "g", "LineWidth", 1.0);
    if has_column(T, "motor_on_detected")
        plot(ax5, t, double(logical(T.motor_on_detected)), "Color", [0.47 0.67 0.19], "LineWidth", 0.8);
        legend(ax5, "PWM mean", "vel active&valid", "motor on", "Location", "eastoutside");
    else
        legend(ax5, "PWM mean", "vel active&valid", "Location", "eastoutside");
    end
    ylabel(ax5, "Flag");
    ylim(ax5, [-0.05 1.2]);
    add_active_background(ax5, t, judge_mask);
    xlabel(ax5, "Time since log start [s]");
    title(ax5, "PWM and validation mask");
    grid(ax5, "on");

    linkaxes([ax1 ax2 ax3 ax4 ax5], "x");
    drawnow;

    if save_plot
        if strlength(string(plot_file)) == 0
            plot_file = replace(string(csv_file), ".CSV", "_vel_validation.png");
            plot_file = replace(string(plot_file), ".csv", "_vel_validation.png");
        end
        try
            saveas(fig, plot_file);
            fprintf("\nSaved plot: %s\n", string(plot_file));
        catch ME
            warning("Could not save plot: %s", ME.message);
        end
    end
end

%% Verdict text
hard_fail = false;
warn_count = 0;
for i = 1:numel(results)
    if results{i}.level == "FAIL"
        hard_fail = true;
    elseif results{i}.level == "WARN"
        warn_count = warn_count + 1;
    end
end

fprintf("\n=== Overall verdict ===\n");
if hard_fail
    fprintf("FAIL: velocity-control core behavior needs attention.\n");
elseif warn_count > 0
    fprintf("WARN: velocity-control core is mostly OK, but test/attitude tracking issues remain.\n");
else
    fprintf("PASS: velocity damping and attitude tracking look acceptable under current limits.\n");
end

fprintf("\nInterpretation guide:\n");
fprintf("- If velocity RMS/P95 and command direction pass, velocity controller sign/output is OK.\n");
fprintf("- If desired tilt is small but actual tilt/error/correlation warn, the issue is likely attitude authority, mounting, tether geometry, or test setup.\n");
fprintf("- If desired tilt is large or saturates, reduce velocity-loop gain/slew/tilt limits.\n");

%% Local functions
function assert_required_columns(T, names)
    vars = string(T.Properties.VariableNames);
    missing = names(~ismember(names, vars));
    if ~isempty(missing)
        error("Missing required columns:\n%s", strjoin(missing, newline));
    end
end

function tf = has_column(T, name)
    tf = ismember(string(name), string(T.Properties.VariableNames));
end

function S = mag_stats(x, m)
    x = x(m);
    x = x(isfinite(x));
    if isempty(x)
        S.mean = NaN; S.rms = NaN; S.p50 = NaN; S.p95 = NaN; S.max = NaN; S.min = NaN;
        return;
    end
    S.mean = mean(x);
    S.rms = sqrt(mean(x.^2));
    S.p50 = finite_percentile(x, 50);
    S.p95 = finite_percentile(x, 95);
    S.max = max(x);
    S.min = min(x);
end

function S = signed_stats(x, m)
    x = x(m);
    x = x(isfinite(x));
    if isempty(x)
        S.mean = NaN; S.std = NaN; S.p05 = NaN; S.p95 = NaN; S.max_abs = NaN; S.rms = NaN; S.p95_abs = NaN;
        return;
    end
    S.mean = mean(x);
    S.std = std(x);
    S.p05 = finite_percentile(x, 5);
    S.p95 = finite_percentile(x, 95);
    S.max_abs = max(abs(x));
    S.rms = sqrt(mean(x.^2));
    S.p95_abs = finite_percentile(abs(x), 95);
end

function S = demean_error_stats(des, actual, m)
    des = des(m);
    actual = actual(m);
    good = isfinite(des) & isfinite(actual);
    des = des(good);
    actual = actual(good);
    des = des - mean(des);
    actual = actual - mean(actual);
    e = actual - des;
    S.rms = sqrt(mean(e.^2));
    S.p95_abs = finite_percentile(abs(e), 95);
end

function r = finite_mean(x)
    x = x(isfinite(x));
    if isempty(x), r = NaN; else, r = mean(x); end
end

function r = finite_median(x)
    x = sort(x(isfinite(x)));
    n = numel(x);
    if n == 0
        r = NaN;
    elseif mod(n, 2) == 1
        r = x((n + 1) / 2);
    else
        r = 0.5 * (x(n / 2) + x(n / 2 + 1));
    end
end

function r = finite_rms(x)
    x = x(isfinite(x));
    if isempty(x), r = NaN; else, r = sqrt(mean(x.^2)); end
end

function r = finite_max(x)
    x = x(isfinite(x));
    if isempty(x), r = NaN; else, r = max(x); end
end

function p = finite_percentile(x, q)
    x = sort(x(isfinite(x)));
    n = numel(x);
    if n == 0
        p = NaN;
        return;
    end
    if n == 1
        p = x(1);
        return;
    end
    q = max(0, min(100, q));
    pos = 1 + (n - 1) * q / 100;
    lo = floor(pos);
    hi = ceil(pos);
    if lo == hi
        p = x(lo);
    else
        w = pos - lo;
        p = (1 - w) * x(lo) + w * x(hi);
    end
end

function c = corr_basic(a, b)
    a = a(:);
    b = b(:);
    good = isfinite(a) & isfinite(b);
    a = a(good);
    b = b(good);
    if numel(a) < 2
        c = NaN;
        return;
    end
    a = a - mean(a);
    b = b - mean(b);
    den = sqrt(sum(a.^2) * sum(b.^2));
    if den <= eps
        c = NaN;
    else
        c = sum(a .* b) / den;
    end
end

function seg = logical_segments(mask)
    mask = logical(mask(:));
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
    seg = [starts ends];
end

function dur = sum_segment_duration(t, seg)
    dur = 0;
    for k = 1:size(seg, 1)
        dur = dur + max(0, t(seg(k, 2)) - t(seg(k, 1)));
    end
end

function R = check_result(name, pass, detail, fail_msg)
    R.name = string(name);
    if pass
        R.level = "PASS";
        R.message = string(detail);
    else
        R.level = "FAIL";
        R.message = string(fail_msg) + " (" + string(detail) + ")";
    end
end

function R = check_warn(name, pass, detail, warn_msg)
    R.name = string(name);
    if pass
        R.level = "PASS";
        R.message = string(detail);
    else
        R.level = "WARN";
        R.message = string(warn_msg) + " (" + string(detail) + ")";
    end
end

function print_results(results)
    fprintf("--- Validation summary ---\n");
    for i = 1:numel(results)
        R = results{i};
        fprintf("[%s] %-26s %s\n", R.level, R.name + ":", R.message);
    end
end

function print_mag(name, S, unit)
    fprintf("%-18s mean/RMS/P50/P95/max = %.4f / %.4f / %.4f / %.4f / %.4f %s\n", ...
        name, S.mean, S.rms, S.p50, S.p95, S.max, unit);
end

function print_signed(name, S, unit)
    fprintf("%-18s mean/std/P05/P95/maxabs = %+8.4f / %.4f / %+8.4f / %+8.4f / %.4f %s\n", ...
        name, S.mean, S.std, S.p05, S.p95, S.max_abs, unit);
end

function add_active_background(ax, t, mask)
    yl = ylim(ax);
    seg = logical_segments(mask);
    for k = 1:size(seg, 1)
        xs = [t(seg(k, 1)) t(seg(k, 2)) t(seg(k, 2)) t(seg(k, 1))];
        ys = [yl(1) yl(1) yl(2) yl(2)];
        h = patch(ax, xs, ys, [0.47 0.67 0.19], ...
            "FaceAlpha", 0.10, "EdgeColor", "none", "HandleVisibility", "off");
        try
            uistack(h, "bottom");
        catch
        end
    end
    ylim(ax, yl);
end
