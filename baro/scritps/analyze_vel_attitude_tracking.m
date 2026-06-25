clear; clc; close all;

%% STRIX velocity damping / attitude tracking check
% Focus:
%   target velocity -> measured/filtered velocity -> attitude command
%   -> actual roll/pitch tracking

csv_file = "data/260620_2.CSV";
if ~isfile(csv_file)
    [f, p] = uigetfile("*.CSV", "Select STRIX log CSV");
    if isequal(f, 0)
        error("No CSV file selected.");
    end
    csv_file = string(fullfile(p, f));
end

T = readtable(csv_file, "VariableNamingRule", "preserve");

t = (T.timestamp_ms - T.timestamp_ms(1)) * 1e-3;
dt = diff(t);
fprintf("=== STRIX Velocity-Attitude Tracking Check ===\n");
fprintf("File      : %s\n", csv_file);
fprintf("Rows      : %d\n", height(T));
fprintf("Duration  : %.2f s\n", t(end));
fprintf("Median Hz : %.2f Hz\n\n", 1 / median(dt, "omitnan"));

active = logical(T.vel_ctrl_active) & logical(T.vel_ctrl_valid);
motor_on = logical(T.motor_on_detected);
ekf_ready = logical(T.ekf_ready);

v_sp_n = T.vel_sp_n_mps;
v_sp_e = T.vel_sp_e_mps;
v_filt_n = T.vel_filt_n_mps;
v_filt_e = T.vel_filt_e_mps;
v_meas_n = T.vel_meas_n_mps;
v_meas_e = T.vel_meas_e_mps;
v_ekf_n = T.ekf_vel_n;
v_ekf_e = T.ekf_vel_e;

v_filt_xy = hypot(v_filt_n, v_filt_e);
v_meas_xy = hypot(v_meas_n, v_meas_e);
v_ekf_xy = hypot(v_ekf_n, v_ekf_e);

acc_cmd_n = T.vel_acc_cmd_n_mps2;
acc_cmd_e = T.vel_acc_cmd_e_mps2;
acc_cmd_xy = hypot(acc_cmd_n, acc_cmd_e);

roll_des = T.roll_des_deg;
pitch_des = T.pitch_des_deg;
att_roll_des = T.att_roll_des_deg;
att_pitch_des = T.att_pitch_des_deg;
roll = T.roll_deg;
pitch = T.pitch_deg;

tilt_des = hypot(roll_des, pitch_des);
tilt_actual = hypot(roll, pitch);

roll_err = roll - roll_des;
pitch_err = pitch - pitch_des;

fprintf("--- Active/valid ratio ---\n");
fprintf("ekf_ready       : %6.2f %%\n", 100 * mean(ekf_ready));
fprintf("motor_on        : %6.2f %%\n", 100 * mean(motor_on));
fprintf("vel_ctrl_active : %6.2f %%\n", 100 * mean(active));
fprintf("acc saturated   : %6.2f %%\n", 100 * mean(logical(T.vel_ctrl_accel_saturated(active))));
fprintf("tilt saturated  : %6.2f %%\n\n", 100 * mean(logical(T.vel_ctrl_tilt_saturated(active))));

print_mag_stats("target |Vxy|", hypot(v_sp_n, v_sp_e), active, "m/s");
print_mag_stats("filtered |Vxy|", v_filt_xy, active, "m/s");
print_mag_stats("measured |Vxy|", v_meas_xy, active, "m/s");
print_mag_stats("EKF |Vxy|", v_ekf_xy, active, "m/s");
fprintf("\n");

print_signed_stats("vel_filt_n", v_filt_n, active, "m/s");
print_signed_stats("vel_filt_e", v_filt_e, active, "m/s");
print_signed_stats("vel_err_n", T.vel_err_n_mps, active, "m/s");
print_signed_stats("vel_err_e", T.vel_err_e_mps, active, "m/s");
print_signed_stats("acc_cmd_n", acc_cmd_n, active, "m/s^2");
print_signed_stats("acc_cmd_e", acc_cmd_e, active, "m/s^2");
fprintf("\n");

print_mag_stats("desired tilt", tilt_des, active, "deg");
print_mag_stats("actual tilt", tilt_actual, active, "deg");
print_signed_stats("roll_des", roll_des, active, "deg");
print_signed_stats("pitch_des", pitch_des, active, "deg");
print_signed_stats("roll actual", roll, active, "deg");
print_signed_stats("pitch actual", pitch, active, "deg");
fprintf("\n");

print_tracking_stats("roll", roll_des, roll, active);
print_tracking_stats("pitch", pitch_des, pitch, active);

fprintf("\n--- Direction sanity check, active only ---\n");
fprintf("corr(v_filt_n, acc_cmd_n): %+7.3f\n", corr_basic(v_filt_n(active), acc_cmd_n(active)));
fprintf("corr(v_filt_e, acc_cmd_e): %+7.3f\n", corr_basic(v_filt_e(active), acc_cmd_e(active)));
fprintf("corr(roll_des demeaned, roll demeaned)  : %+7.3f\n", corr_demean(roll_des(active), roll(active)));
fprintf("corr(pitch_des demeaned, pitch demeaned): %+7.3f\n", corr_demean(pitch_des(active), pitch(active)));

fprintf("\n--- Velocity threshold, filtered |Vxy| active only ---\n");
for th = [0.05 0.10 0.15 0.20]
    fprintf("|Vxy| < %.2f m/s : %6.2f %%\n", th, 100 * mean(v_filt_xy(active) < th));
end

%% Plots
fig = figure("Name", "STRIX Velocity-Attitude Tracking", "Color", "w");
fig.Position = [80 60 1450 950];
tl = tiledlayout(fig, 5, 1, "TileSpacing", "compact", "Padding", "compact");
title(tl, "STRIX velocity damping / attitude tracking");

ax1 = nexttile;
hold(ax1, "on");
plot(ax1, t, v_sp_n, "k--", "LineWidth", 1.0);
plot(ax1, t, v_sp_e, "--", "Color", [0.35 0.35 0.35], "LineWidth", 1.0);
plot(ax1, t, v_filt_n, "b", "LineWidth", 1.1);
plot(ax1, t, v_filt_e, "Color", [0.85 0.33 0.10], "LineWidth", 1.1);
plot_active_background(ax1, t, active);
ylabel(ax1, "Velocity [m/s]");
title(ax1, "Target velocity vs filtered current velocity");
legend(ax1, "target N", "target E", "filtered N", "filtered E", "Location", "eastoutside");
grid(ax1, "on");

ax2 = nexttile;
hold(ax2, "on");
plot(ax2, t, v_filt_xy, "Color", [0.49 0.18 0.56], "LineWidth", 1.2);
plot(ax2, t, v_meas_xy, "Color", [0.93 0.69 0.13], "LineWidth", 0.9);
yline(ax2, 0.10, "g--", "0.10 m/s");
yline(ax2, 0.15, "r--", "0.15 m/s");
plot_active_background(ax2, t, active);
ylabel(ax2, "|Vxy| [m/s]");
title(ax2, "Horizontal speed magnitude");
legend(ax2, "filtered |Vxy|", "measured |Vxy|", "Location", "eastoutside");
grid(ax2, "on");

ax3 = nexttile;
hold(ax3, "on");
plot(ax3, t, acc_cmd_n, "b", "LineWidth", 1.0);
plot(ax3, t, acc_cmd_e, "Color", [0.85 0.33 0.10], "LineWidth", 1.0);
plot(ax3, t, acc_cmd_xy, "r", "LineWidth", 0.9);
plot_active_background(ax3, t, active);
ylabel(ax3, "Accel cmd [m/s^2]");
title(ax3, "Velocity-error acceleration command");
legend(ax3, "acc cmd N", "acc cmd E", "|acc cmd|", "Location", "eastoutside");
grid(ax3, "on");

ax4 = nexttile;
hold(ax4, "on");
plot(ax4, t, roll_des, "b--", "LineWidth", 1.2);
plot(ax4, t, pitch_des, "--", "Color", [0.85 0.33 0.10], "LineWidth", 1.2);
plot(ax4, t, roll, "b", "LineWidth", 0.9);
plot(ax4, t, pitch, "Color", [0.85 0.33 0.10], "LineWidth", 0.9);
plot_active_background(ax4, t, active);
ylabel(ax4, "Attitude [deg]");
title(ax4, "Desired attitude from velocity controller vs actual attitude");
legend(ax4, "roll des", "pitch des", "roll actual", "pitch actual", "Location", "eastoutside");
grid(ax4, "on");

ax5 = nexttile;
hold(ax5, "on");
yyaxis(ax5, "left");
plot(ax5, t, T.ekf_pwm_mean, "Color", [0.25 0.25 0.25], "LineWidth", 1.1);
ylabel(ax5, "PWM [us]");
yyaxis(ax5, "right");
plot(ax5, t, double(active), "g", "LineWidth", 1.0);
plot(ax5, t, double(motor_on), "Color", [0.47 0.67 0.19], "LineWidth", 0.8);
ylabel(ax5, "Flag");
ylim(ax5, [-0.05 1.2]);
plot_active_background(ax5, t, active);
xlabel(ax5, "Time since log start [s]");
title(ax5, "PWM and active windows");
legend(ax5, "PWM mean", "vel active&valid", "motor on", "Location", "eastoutside");
grid(ax5, "on");

linkaxes([ax1 ax2 ax3 ax4 ax5], "x");
drawnow;

out_png = replace(csv_file, ".CSV", "_vel_att_tracking.png");
try
    exportgraphics(fig, out_png, "Resolution", 180);
    fprintf("\nSaved plot: %s\n", out_png);
catch
    warning("Could not export plot PNG. Figure is still open.");
end

%% Per-active-segment summary
fprintf("\n--- Active segments ---\n");
segments = logical_segments(active);
for i = 1:size(segments, 1)
    s = segments(i, 1);
    e = segments(i, 2);
    m = false(height(T), 1);
    m(s:e) = true;

    fprintf("seg %d: %.2f ~ %.2f s, dur %.2f s, pwm mean %.0f\n", ...
        i, t(s), t(e), t(e) - t(s), mean(T.ekf_pwm_mean(m), "omitnan"));
    fprintf("  filtered |Vxy| mean/RMS/P95/max = %.3f / %.3f / %.3f / %.3f m/s\n", ...
        mean(v_filt_xy(m), "omitnan"), rms_value(v_filt_xy(m)), prctile(v_filt_xy(m), 95), max(v_filt_xy(m)));
    fprintf("  desired tilt P95/max = %.2f / %.2f deg\n", ...
        prctile(tilt_des(m), 95), max(tilt_des(m)));
    fprintf("  actual  tilt P95/max = %.2f / %.2f deg\n", ...
        prctile(tilt_actual(m), 95), max(tilt_actual(m)));
    fprintf("  roll err RMS/P95abs = %.2f / %.2f deg\n", ...
        rms_value(roll_err(m)), prctile(abs(roll_err(m)), 95));
    fprintf("  pitch err RMS/P95abs = %.2f / %.2f deg\n", ...
        rms_value(pitch_err(m)), prctile(abs(pitch_err(m)), 95));
end

%% Local functions
function print_mag_stats(name, x, m, unit)
    x = x(m);
    fprintf("%-18s mean/RMS/P50/P95/max = %.4f / %.4f / %.4f / %.4f / %.4f %s\n", ...
        name, mean(x, "omitnan"), rms_value(x), median(x, "omitnan"), prctile(x, 95), max(x), unit);
end

function print_signed_stats(name, x, m, unit)
    x = x(m);
    fprintf("%-18s mean/std/P05/P95/maxabs = %+8.4f / %.4f / %+8.4f / %+8.4f / %.4f %s\n", ...
        name, mean(x, "omitnan"), std(x, "omitnan"), prctile(x, 5), prctile(x, 95), max(abs(x)), unit);
end

function print_tracking_stats(name, des, actual, m)
    e = actual(m) - des(m);
    des0 = des(m) - mean(des(m), "omitnan");
    actual0 = actual(m) - mean(actual(m), "omitnan");
    e0 = actual0 - des0;
    fprintf("%s tracking raw err mean/RMS/P95abs/maxabs = %+7.3f / %.3f / %.3f / %.3f deg\n", ...
        name, mean(e, "omitnan"), rms_value(e), prctile(abs(e), 95), max(abs(e)));
    fprintf("%s tracking demeaned err RMS/P95abs/corr = %.3f / %.3f / %+7.3f\n", ...
        name, rms_value(e0), prctile(abs(e0), 95), corr_demean(des(m), actual(m)));
end

function r = rms_value(x)
    x = x(isfinite(x));
    r = sqrt(mean(x.^2));
end

function c = corr_demean(a, b)
    a = a(:);
    b = b(:);
    good = isfinite(a) & isfinite(b);
    a = a(good) - mean(a(good), "omitnan");
    b = b(good) - mean(b(good), "omitnan");
    c = corr_basic(a, b);
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

function plot_active_background(ax, t, active)
    active = active(:);
    seg = logical_segments(active);
    yl = ylim(ax);
    hold(ax, "on");
    for k = 1:size(seg, 1)
        xs = [t(seg(k, 1)) t(seg(k, 2)) t(seg(k, 2)) t(seg(k, 1))];
        ys = [yl(1) yl(1) yl(2) yl(2)];
        h = patch(ax, xs, ys, [0.47 0.67 0.19], "FaceAlpha", 0.10, ...
            "EdgeColor", "none", "HandleVisibility", "off");
        try
            uistack(h, "bottom");
        catch
        end
    end
    ylim(ax, yl);
end

function seg = logical_segments(mask)
    mask = logical(mask(:));
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
    seg = [starts ends];
end
