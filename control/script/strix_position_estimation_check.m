%% STRIX horizontal position estimation / PosHold separation check
% This script checks horizontal EKF position behavior and PosHold performance.
%
% Important limitation:
% - This log has no independent GNSS/local-position reference columns for N/E.
% - Therefore this script does NOT prove absolute position accuracy.
% - It checks internal consistency, drift/hold behavior, setpoint latch stability,
%   and whether EKF horizontal position agrees with logged filtered velocity.

clear; clc; close all;

%% User settings
csv_file = "";
if strlength(csv_file) == 0
    csv_file = string(getenv("STRIX_POS_CSV"));
end
if strlength(csv_file) == 0
    candidates = [
        "pos_test_02.CSV"
        fullfile(pwd, "pos_test_02.CSV")
        fullfile(pwd, ".cache", "pos_test_02.CSV")
    ];
    for k = 1:numel(candidates)
        if isfile(candidates(k))
            csv_file = candidates(k);
            break;
        end
    end
end
if strlength(csv_file) == 0 || ~isfile(csv_file)
    [f, p] = uigetfile({"*.csv;*.CSV", "CSV files"}, "Select STRIX position log");
    if isequal(f, 0)
        error("No CSV selected.");
    end
    csv_file = string(fullfile(p, f));
end

% Judge thresholds. Adjust these after you have several clean logs.
MIN_JUDGE_SEC = 5.0;
POS_ERR_RMS_GOOD_M = 0.50;
POS_ERR_P95_GOOD_M = 0.90;
SPEED_RMS_GOOD_MPS = 0.20;
VEL_POS_DIFF_RMS_GOOD_MPS = 0.20;
SP_STABLE_LIMIT_M = 0.05;

%% Load
opts = detectImportOptions(csv_file, "VariableNamingRule", "preserve");
T = readtable(csv_file, opts);
names = string(T.Properties.VariableNames);

must_have = ["timestamp_ms", "ekf_pos_n", "ekf_pos_e"];
for k = 1:numel(must_have)
    if ~any(names == must_have(k))
        error("Required column missing: %s", must_have(k));
    end
end

t = getcol(T, "timestamp_ms");
t = (t - t(1)) * 1e-3;
dt = diff(t);
dt_good = dt(isfinite(dt) & dt > 0);
median_dt = median(dt_good);
median_rate_hz = 1 / median_dt;
duration_s = t(end) - t(1);

pos_n = getcol(T, "ekf_pos_n");
pos_e = getcol(T, "ekf_pos_e");
ekf_ready = getbool(T, "ekf_ready", true(height(T), 1));
finite_pos = isfinite(pos_n) & isfinite(pos_e);

pos_hold_latched = getbool(T, "pos_hold_latched", false(height(T), 1));
vel_ctrl_active = getbool(T, "vel_ctrl_active", false(height(T), 1));
alt_ctrl_active = getbool(T, "alt_ctrl_active", false(height(T), 1));
motor_on = getbool(T, "motor_on_detected", false(height(T), 1));
motor_allowed = getbool(T, "motor_output_allowed", false(height(T), 1));

pos_judge = ekf_ready & finite_pos & pos_hold_latched;
ekf_judge = ekf_ready & finite_pos;
if nnz(pos_judge) * median_dt < MIN_JUDGE_SEC
    warning("PosHold judge duration is short. Figures still use available data.");
end

%% Optional columns
has_sp = hascol(T, "pos_sp_n_m") && hascol(T, "pos_sp_e_m");
has_err = hascol(T, "pos_err_n_m") && hascol(T, "pos_err_e_m");
has_vel = hascol(T, "vel_filt_n_mps") && hascol(T, "vel_filt_e_mps");
has_att = hascol(T, "roll_deg") && hascol(T, "pitch_deg") && ...
          hascol(T, "roll_des_slew_deg") && hascol(T, "pitch_des_slew_deg");
has_pwm = all(arrayfun(@(s) hascol(T, s), ["M1", "M2", "M3", "M4"]));

if has_sp
    sp_n = getcol(T, "pos_sp_n_m");
    sp_e = getcol(T, "pos_sp_e_m");
else
    sp_n = nan(height(T), 1);
    sp_e = nan(height(T), 1);
end

if has_err
    log_err_n = getcol(T, "pos_err_n_m");
    log_err_e = getcol(T, "pos_err_e_m");
elseif has_sp
    log_err_n = sp_n - pos_n;
    log_err_e = sp_e - pos_e;
else
    log_err_n = nan(height(T), 1);
    log_err_e = nan(height(T), 1);
end

if has_vel
    vel_n = getcol(T, "vel_filt_n_mps");
    vel_e = getcol(T, "vel_filt_e_mps");
else
    vel_n = nan(height(T), 1);
    vel_e = nan(height(T), 1);
end

%% Position/velocity consistency
vel_from_pos_n = nan(height(T), 1);
vel_from_pos_e = nan(height(T), 1);
if height(T) >= 3
    vel_from_pos_n(2:end-1) = (pos_n(3:end) - pos_n(1:end-2)) ./ (t(3:end) - t(1:end-2));
    vel_from_pos_e(2:end-1) = (pos_e(3:end) - pos_e(1:end-2)) ./ (t(3:end) - t(1:end-2));
end

vel_pos_diff = hypot(vel_from_pos_n - vel_n, vel_from_pos_e - vel_e);
speed_xy = hypot(vel_n, vel_e);
pos_err_xy = hypot(log_err_n, log_err_e);

%% Summary print
fprintf("\n=== STRIX Horizontal Position / PosHold Check ===\n");
fprintf("File        : %s\n", csv_file);
fprintf("Rows        : %d\n", height(T));
fprintf("Duration    : %.2f s\n", duration_s);
fprintf("Median rate : %.2f Hz\n", median_rate_hz);
fprintf("Note        : No independent horizontal reference in this CSV.\n");
fprintf("              This checks consistency/hold behavior, not absolute accuracy.\n\n");

fprintf("--- Availability ---\n");
print_ratio("ekf_ready", ekf_ready);
print_ratio("pos_hold_latched", pos_hold_latched);
print_ratio("vel_ctrl_active", vel_ctrl_active);
print_ratio("alt_ctrl_active", alt_ctrl_active);
print_ratio("motor_on_detected", motor_on);
print_ratio("motor_output_allowed", motor_allowed);
fprintf("Pos judge samples/duration: %d / %.2f s\n", nnz(pos_judge), nnz(pos_judge) * median_dt);
fprintf("EKF judge samples/duration: %d / %.2f s\n\n", nnz(ekf_judge), nnz(ekf_judge) * median_dt);

fprintf("--- EKF horizontal movement, EKF-ready mask ---\n");
print_motion_summary(t, pos_n, pos_e, ekf_judge);

fprintf("\n--- PosHold movement, PosHold latched + EKF-ready mask ---\n");
print_motion_summary(t, pos_n, pos_e, pos_judge);

if has_sp
    sp_span_n = span_finite(sp_n(pos_judge));
    sp_span_e = span_finite(sp_e(pos_judge));
    fprintf("\n--- PosHold setpoint stability ---\n");
    fprintf("pos_sp_n/e span during PosHold: %.4f / %.4f m\n", sp_span_n, sp_span_e);
    print_pass("setpoint stable", max(sp_span_n, sp_span_e) <= SP_STABLE_LIMIT_M, ...
        sprintf("max span %.4f m <= %.4f m", max(sp_span_n, sp_span_e), SP_STABLE_LIMIT_M));
end

if has_err || has_sp
    fprintf("\n--- Position hold error ---\n");
    print_metric_block("pos_err_xy", pos_err_xy(pos_judge), "m");
    print_pass("pos_err RMS", rms_finite(pos_err_xy(pos_judge)) <= POS_ERR_RMS_GOOD_M, ...
        sprintf("%.3f m <= %.3f m", rms_finite(pos_err_xy(pos_judge)), POS_ERR_RMS_GOOD_M));
    print_pass("pos_err P95", pctl_abs(pos_err_xy(pos_judge), 95) <= POS_ERR_P95_GOOD_M, ...
        sprintf("%.3f m <= %.3f m", pctl_abs(pos_err_xy(pos_judge), 95), POS_ERR_P95_GOOD_M));

    if has_sp && has_err && nnz(pos_judge) > 0
        diff_sp_minus_pos = mean(abs(log_err_n(pos_judge) - (sp_n(pos_judge) - pos_n(pos_judge))) + ...
                                abs(log_err_e(pos_judge) - (sp_e(pos_judge) - pos_e(pos_judge))), "omitnan");
        diff_pos_minus_sp = mean(abs(log_err_n(pos_judge) - (pos_n(pos_judge) - sp_n(pos_judge))) + ...
                                abs(log_err_e(pos_judge) - (pos_e(pos_judge) - sp_e(pos_judge))), "omitnan");
        if diff_sp_minus_pos < diff_pos_minus_sp
            fprintf("pos_err convention: pos_err ~= pos_sp - ekf_pos\n");
        else
            fprintf("pos_err convention: pos_err ~= ekf_pos - pos_sp\n");
        end
    end
end

if has_vel
    fprintf("\n--- Horizontal velocity / position consistency ---\n");
    print_metric_block("speed_xy", speed_xy(pos_judge), "m/s");
    print_metric_block("abs(dpos/dt - vel_filt)", vel_pos_diff(pos_judge), "m/s");
    print_pass("speed RMS", rms_finite(speed_xy(pos_judge)) <= SPEED_RMS_GOOD_MPS, ...
        sprintf("%.3f m/s <= %.3f m/s", rms_finite(speed_xy(pos_judge)), SPEED_RMS_GOOD_MPS));
    print_pass("vel-pos diff RMS", rms_finite(vel_pos_diff(pos_judge)) <= VEL_POS_DIFF_RMS_GOOD_MPS, ...
        sprintf("%.3f m/s <= %.3f m/s", rms_finite(vel_pos_diff(pos_judge)), VEL_POS_DIFF_RMS_GOOD_MPS));
end

%% Figures
figure("Name", "Horizontal trajectory");
tiledlayout(1, 1, "TileSpacing", "compact", "Padding", "compact");
nexttile;
plot(pos_e(ekf_judge), pos_n(ekf_judge), "-", "LineWidth", 1.2); hold on; grid on; axis equal;
if has_sp && any(pos_judge)
    plot(sp_e(pos_judge), sp_n(pos_judge), "r.", "MarkerSize", 10);
    plot(pos_e(pos_judge), pos_n(pos_judge), "k-", "LineWidth", 1.8);
    legend("EKF-ready trajectory", "PosHold setpoint", "PosHold trajectory", "Location", "best");
else
    legend("EKF-ready trajectory", "Location", "best");
end
xlabel("East position [m]");
ylabel("North position [m]");
title("Horizontal EKF trajectory");

figure("Name", "Position states and PosHold error");
tiledlayout(4, 1, "TileSpacing", "compact", "Padding", "compact");
nexttile;
plot(t, pos_n, "LineWidth", 1.1); hold on; grid on;
if has_sp, plot(t, sp_n, "--", "LineWidth", 1.0); end
ylabel("N [m]");
title("North position");
legend_if(has_sp, ["ekf_pos_n", "pos_sp_n"], ["ekf_pos_n"]);

nexttile;
plot(t, pos_e, "LineWidth", 1.1); hold on; grid on;
if has_sp, plot(t, sp_e, "--", "LineWidth", 1.0); end
ylabel("E [m]");
title("East position");
legend_if(has_sp, ["ekf_pos_e", "pos_sp_e"], ["ekf_pos_e"]);

nexttile;
if has_err || has_sp
    plot(t, log_err_n, "LineWidth", 1.1); hold on;
    plot(t, log_err_e, "LineWidth", 1.1);
    plot(t, pos_err_xy, "k", "LineWidth", 1.2);
end
grid on; ylabel("error [m]");
title("PosHold error");
legend_if(has_err || has_sp, ["err_N", "err_E", "err_XY"], ["no error columns"]);

nexttile;
stairs(t, double(ekf_ready), "LineWidth", 1.0); hold on; grid on;
stairs(t, double(pos_hold_latched), "LineWidth", 1.0);
stairs(t, double(vel_ctrl_active), "LineWidth", 1.0);
ylabel("flag"); xlabel("time [s]");
title("Mode flags");
legend("ekf_ready", "pos_hold_latched", "vel_ctrl_active", "Location", "best");

if has_vel
    figure("Name", "Velocity consistency");
    tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");
    nexttile;
    plot(t, vel_n, "LineWidth", 1.1); hold on; grid on;
    plot(t, vel_from_pos_n, "--", "LineWidth", 1.0);
    ylabel("N vel [m/s]");
    title("North velocity: logged vs d(position)/dt");
    legend("vel_filt_n", "d(ekf_pos_n)/dt", "Location", "best");

    nexttile;
    plot(t, vel_e, "LineWidth", 1.1); hold on; grid on;
    plot(t, vel_from_pos_e, "--", "LineWidth", 1.0);
    ylabel("E vel [m/s]");
    title("East velocity: logged vs d(position)/dt");
    legend("vel_filt_e", "d(ekf_pos_e)/dt", "Location", "best");

    nexttile;
    plot(t, speed_xy, "LineWidth", 1.1); hold on; grid on;
    plot(t, vel_pos_diff, "LineWidth", 1.1);
    ylabel("[m/s]"); xlabel("time [s]");
    title("Speed and velocity-position mismatch");
    legend("speed_xy", "|dpos/dt - vel_filt|", "Location", "best");
end

if has_att || has_pwm
    figure("Name", "Control context");
    ntiles = double(has_att) + double(has_pwm);
    tiledlayout(ntiles, 1, "TileSpacing", "compact", "Padding", "compact");
    if has_att
        nexttile;
        plot(t, getcol(T, "roll_deg"), "LineWidth", 1.0); hold on; grid on;
        plot(t, getcol(T, "roll_des_slew_deg"), "--", "LineWidth", 1.0);
        plot(t, getcol(T, "pitch_deg"), "LineWidth", 1.0);
        plot(t, getcol(T, "pitch_des_slew_deg"), "--", "LineWidth", 1.0);
        ylabel("angle [deg]");
        title("Attitude tracking context");
        legend("roll", "roll des", "pitch", "pitch des", "Location", "best");
    end
    if has_pwm
        nexttile;
        plot(t, getcol(T, "M1"), "LineWidth", 0.9); hold on; grid on;
        plot(t, getcol(T, "M2"), "LineWidth", 0.9);
        plot(t, getcol(T, "M3"), "LineWidth", 0.9);
        plot(t, getcol(T, "M4"), "LineWidth", 0.9);
        ylabel("PWM [us]"); xlabel("time [s]");
        title("Motor PWM context");
        legend("M1", "M2", "M3", "M4", "Location", "best");
    end
end

fprintf("\nFigures generated.\n");

%% Local functions
function tf = hascol(T, name)
    tf = any(string(T.Properties.VariableNames) == string(name));
end

function x = getcol(T, name)
    names = string(T.Properties.VariableNames);
    idx = find(names == string(name), 1);
    if isempty(idx)
        error("Missing column: %s", name);
    end
    x = T{:, idx};
    if iscell(x)
        x = str2double(string(x));
    end
    x = double(x);
end

function b = getbool(T, name, default_value)
    if hascol(T, name)
        x = getcol(T, name);
        b = x ~= 0 & isfinite(x);
    else
        b = default_value;
    end
end

function y = rms_finite(x)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = sqrt(mean(x.^2));
    end
end

function y = pctl_abs(x, p)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = prctile(abs(x), p);
    end
end

function y = span_finite(x)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = max(x) - min(x);
    end
end

function print_ratio(label, mask)
    fprintf("%-22s: %7.2f %%\n", label, 100 * mean(mask));
end

function print_metric_block(label, x, unit)
    x = x(isfinite(x));
    if isempty(x)
        fprintf("%-24s: no finite samples\n", label);
        return;
    end
    fprintf("%-24s mean/rms/P95/max = %+.4f / %.4f / %.4f / %.4f %s\n", ...
        label, mean(x), sqrt(mean(x.^2)), prctile(abs(x), 95), max(abs(x)), unit);
end

function print_pass(label, pass, detail)
    if pass
        tag = "PASS";
    else
        tag = "WARN";
    end
    fprintf("[%s] %-18s: %s\n", tag, label, detail);
end

function print_motion_summary(t, n, e, mask)
    idx = find(mask & isfinite(n) & isfinite(e));
    if numel(idx) < 2
        fprintf("Not enough samples.\n");
        return;
    end
    nn = n(idx);
    ee = e(idx);
    dur = t(idx(end)) - t(idx(1));
    dn = nn(end) - nn(1);
    de = ee(end) - ee(1);
    fprintf("duration                       : %.2f s\n", dur);
    fprintf("start N/E                      : %+8.4f / %+8.4f m\n", nn(1), ee(1));
    fprintf("end   N/E                      : %+8.4f / %+8.4f m\n", nn(end), ee(end));
    fprintf("final displacement N/E/XY      : %+8.4f / %+8.4f / %8.4f m\n", dn, de, hypot(dn, de));
    fprintf("position span N/E              : %8.4f / %8.4f m\n", max(nn) - min(nn), max(ee) - min(ee));
end

function legend_if(cond, labels_true, labels_false)
    if cond
        legend(labels_true, "Location", "best");
    else
        legend(labels_false, "Location", "best");
    end
end
