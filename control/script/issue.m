%% STRIX PosHold incident analyzer
% Analyze one PosHold flight log and separate:
% 1) position/GNSS/EKF consistency,
% 2) initial body-left drift,
% 3) recovery and overshoot,
% 4) yaw / attitude tracking problems,
% 5) motor output context.
%
% Usage:
%   1. Put the CSV in the same folder, or edit csv_file below.
%   2. Run this script in MATLAB.
%
% Notes:
% - NED convention: N positive north, E positive east.
% - yaw_deg is assumed to be degrees clockwise from North in NED.
% - Body FRD: X forward, Y right. Body-left displacement = -body_y.

clear; clc; close all;

%% User settings
csv_file = "";
if strlength(csv_file) == 0
    csv_file = string(getenv("STRIX_POSHOLD_CSV"));
end
if strlength(csv_file) == 0
    candidates = [
        "real_03(1).CSV"
        "real_03.CSV"
        fullfile(pwd, ".cache", "real_03(1).CSV")
        fullfile(pwd, ".cache", "real_03.CSV")
        "/workspace/.cache/real_03(1).CSV"
        "/workspace/.cache/real_03.CSV"
    ];
    hit = candidates(isfile(candidates));
    if ~isempty(hit)
        csv_file = hit(1);
    end
end
if strlength(csv_file) == 0 || ~isfile(csv_file)
    [f, p] = uigetfile("*.CSV;*.csv", "Select STRIX CSV log");
    if isequal(f, 0)
        error("No CSV selected.");
    end
    csv_file = string(fullfile(p, f));
end

save_figures = false;
out_dir = fullfile(pwd, "poshold_issue_figures");

% Thresholds used for printed judgements.
yaw_bad_deg = 10.0;
yaw_very_bad_deg = 25.0;
near_target_m = 0.20;
near_target_speed_mps = 0.20;
pos_sp_drift_warn_m = 0.05;
ekf_gnss_warn_m = 0.50;

%% Load
opts = detectImportOptions(csv_file, "VariableNamingRule", "preserve");
T = readtable(csv_file, opts);
names = string(T.Properties.VariableNames);

required = [
    "timestamp_ms", "ekf_pos_n", "ekf_pos_e", ...
    "gnss_pos_n_m", "gnss_pos_e_m", ...
    "roll_deg", "pitch_deg", "yaw_deg", ...
    "roll_des_slew_deg", "pitch_des_slew_deg", ...
    "pos_hold_latched", "pos_sp_n_m", "pos_sp_e_m", ...
    "pos_err_n_m", "pos_err_e_m", ...
    "vel_sp_n_mps", "vel_sp_e_mps", ...
    "vel_filt_n_mps", "vel_filt_e_mps", ...
    "yaw_des_deg", "yaw_err_deg", "yaw_rate_sp_deg_s", ...
    "M1", "M2", "M3", "M4"
];
missing = required(~ismember(required, names));
if ~isempty(missing)
    error("Missing required columns: %s", strjoin(missing, ", "));
end

t = (T.timestamp_ms - T.timestamp_ms(1)) / 1000.0;
dt = diff(t);
fs_med = 1 / median(dt(dt > 0), "omitnan");

m_pos = logical(T.pos_hold_latched);
m_vel = get_logical(T, "vel_ctrl_active");
m_alt = get_logical(T, "alt_ctrl_active");
m_ekf = get_logical(T, "ekf_ready");

pos_n = T.ekf_pos_n;
pos_e = T.ekf_pos_e;
gnss_n = T.gnss_pos_n_m;
gnss_e = T.gnss_pos_e_m;

pos_err_n = T.pos_err_n_m;
pos_err_e = T.pos_err_e_m;
pos_err_mag = hypot(pos_err_n, pos_err_e);

vel_n = T.vel_filt_n_mps;
vel_e = T.vel_filt_e_mps;
vel_mag = hypot(vel_n, vel_e);

vel_sp_n = T.vel_sp_n_mps;
vel_sp_e = T.vel_sp_e_mps;
vel_err_n = get_col(T, "vel_err_n_mps", vel_sp_n - vel_n);
vel_err_e = get_col(T, "vel_err_e_mps", vel_sp_e - vel_e);
vel_err_mag = hypot(vel_err_n, vel_err_e);

roll = T.roll_deg;
pitch = T.pitch_deg;
yaw = T.yaw_deg;
yaw_des = T.yaw_des_deg;
yaw_err = T.yaw_err_deg;
yaw_rate_sp = T.yaw_rate_sp_deg_s;
roll_des = T.roll_des_slew_deg;
pitch_des = T.pitch_des_slew_deg;

roll_track_err = roll_des - roll;
pitch_track_err = pitch_des - pitch;

M = [T.M1, T.M2, T.M3, T.M4];
pwm_mean = mean(M, 2, "omitnan");
pwm_span = max(M, [], 2) - min(M, [], 2);

%% PosHold interval and body-frame projection
seg_pos = mask_segments(m_pos);
if isempty(seg_pos)
    error("No pos_hold_latched segment found.");
end

% Use the longest PosHold segment as the incident segment.
[~, longest_idx] = max(seg_pos(:, 2) - seg_pos(:, 1) + 1);
i0 = seg_pos(longest_idx, 1);
i1 = seg_pos(longest_idx, 2);
m_inc = false(height(T), 1);
m_inc(i0:i1) = true;

psi0 = deg2rad(yaw(i0));
body_forward_ne = [cos(psi0), sin(psi0)];       % FRD +X in N/E
body_left_ne = [sin(psi0), -cos(psi0)];         % FRD -Y in N/E

dN = pos_n - pos_n(i0);
dE = pos_e - pos_e(i0);
body_forward_disp = dN * body_forward_ne(1) + dE * body_forward_ne(2);
body_left_disp = dN * body_left_ne(1) + dE * body_left_ne(2);

[peak_left, i_peak_left_rel] = max(body_left_disp(i0:i1));
i_peak_left = i0 + i_peak_left_rel - 1;
[min_after_peak, i_min_after_rel] = min(body_left_disp(i_peak_left:i1));
i_min_after = i_peak_left + i_min_after_rel - 1;

late = find(m_inc & t >= t(i0) + 0.5 * (t(i1) - t(i0)));
if isempty(late)
    late = i0:i1;
end
[best_err_late, best_late_rel] = min(pos_err_mag(late));
i_best_late = late(best_late_rel);

% First threshold crossing after initial peak.
i_recover_1m = first_cross_after(body_left_disp, i_peak_left, i1, 1.0, "below");
i_recover_05m = first_cross_after(body_left_disp, i_peak_left, i1, 0.5, "below");
i_cross_center = first_cross_after(body_left_disp, i_peak_left, i1, 0.0, "below");

%% Summary metrics
pos_sp_drift = hypot(max(T.pos_sp_n_m(i0:i1)) - min(T.pos_sp_n_m(i0:i1)), ...
                     max(T.pos_sp_e_m(i0:i1)) - min(T.pos_sp_e_m(i0:i1)));

ekf_gnss_n_diff = pos_n - gnss_n;
ekf_gnss_e_diff = pos_e - gnss_e;
ekf_gnss_hdiff = hypot(ekf_gnss_n_diff, ekf_gnss_e_diff);

yaw_abs = abs(yaw_err);
yaw_bad_ratio = mean(yaw_abs(m_inc) > yaw_bad_deg, "omitnan");
yaw_very_bad_ratio = mean(yaw_abs(m_inc) > yaw_very_bad_deg, "omitnan");
yaw_rate_sat_abs = max(abs(yaw_rate_sp(m_inc)), [], "omitnan");
yaw_rate_sat_ratio = mean(abs(yaw_rate_sp(m_inc)) > 0.95 * yaw_rate_sat_abs, "omitnan");

near_target = m_inc & pos_err_mag < near_target_m;
near_target_bad_speed_ratio = mean(vel_mag(near_target) > near_target_speed_mps, "omitnan");

%% Print report
fprintf("\n=== STRIX PosHold Incident Analysis ===\n");
fprintf("File             : %s\n", csv_file);
fprintf("Rows             : %d\n", height(T));
fprintf("Duration         : %.2f s\n", t(end));
fprintf("Median rate      : %.2f Hz\n", fs_med);
fprintf("PosHold segment  : %.3f -> %.3f s, %.2f s, %d samples\n", ...
    t(i0), t(i1), t(i1) - t(i0), i1 - i0 + 1);
fprintf("Yaw at latch     : %.2f deg\n", yaw(i0));
fprintf("Body-left dir NE : [N %.3f, E %.3f]\n", body_left_ne(1), body_left_ne(2));

fprintf("\n--- Setpoint / estimator sanity ---\n");
fprintf("Pos SP N/E at latch       : %.3f / %.3f m\n", T.pos_sp_n_m(i0), T.pos_sp_e_m(i0));
fprintf("Pos SP drift during hold  : %.3f m -> %s\n", ...
    pos_sp_drift, pass_fail(pos_sp_drift <= pos_sp_drift_warn_m));
fprintf("EKF-GNSS horizontal diff  : mean %.3f, p95 %.3f, max %.3f m -> %s\n", ...
    mean(ekf_gnss_hdiff(m_inc), "omitnan"), pctl(ekf_gnss_hdiff(m_inc), 95), ...
    max(ekf_gnss_hdiff(m_inc), [], "omitnan"), ...
    pass_fail(max(ekf_gnss_hdiff(m_inc), [], "omitnan") <= ekf_gnss_warn_m));

fprintf("\n--- Initial body-left drift / recovery ---\n");
fprintf("Peak body-left drift      : %.3f m at %.3f s\n", peak_left, t(i_peak_left));
fprintf("  At peak, pos err N/E/mag: %.3f / %.3f / %.3f m\n", ...
    pos_err_n(i_peak_left), pos_err_e(i_peak_left), pos_err_mag(i_peak_left));
fprintf("Best late recovery        : err %.3f m at %.3f s\n", best_err_late, t(i_best_late));
fprintf("End of PosHold            : body-left %.3f m, err %.3f m\n", ...
    body_left_disp(i1), pos_err_mag(i1));
print_cross("Body-left <= 1.0 m after peak", i_recover_1m, t, body_left_disp, pos_err_mag);
print_cross("Body-left <= 0.5 m after peak", i_recover_05m, t, body_left_disp, pos_err_mag);
print_cross("Body-left crossed center", i_cross_center, t, body_left_disp, pos_err_mag);

fprintf("\n--- Yaw / attitude tracking ---\n");
fprintf("Yaw err abs during hold   : mean %.2f, p95 %.2f, max %.2f deg\n", ...
    mean(yaw_abs(m_inc), "omitnan"), pctl(yaw_abs(m_inc), 95), max(yaw_abs(m_inc), [], "omitnan"));
fprintf("Yaw bad ratio > %.1f deg  : %.1f %%\n", yaw_bad_deg, 100 * yaw_bad_ratio);
fprintf("Yaw very bad > %.1f deg   : %.1f %%\n", yaw_very_bad_deg, 100 * yaw_very_bad_ratio);
fprintf("Yaw-rate-sp max abs       : %.2f deg/s, near-max ratio %.1f %%\n", ...
    yaw_rate_sat_abs, 100 * yaw_rate_sat_ratio);
fprintf("Roll tracking err RMS/P95 : %.2f / %.2f deg\n", ...
    rms_omitnan(roll_track_err(m_inc)), pctl(abs(roll_track_err(m_inc)), 95));
fprintf("Pitch tracking err RMS/P95: %.2f / %.2f deg\n", ...
    rms_omitnan(pitch_track_err(m_inc)), pctl(abs(pitch_track_err(m_inc)), 95));

fprintf("\n--- Velocity damping context ---\n");
fprintf("Vel err mag during hold   : mean %.3f, p95 %.3f, max %.3f m/s\n", ...
    mean(vel_err_mag(m_inc), "omitnan"), pctl(vel_err_mag(m_inc), 95), ...
    max(vel_err_mag(m_inc), [], "omitnan"));
if any(near_target)
    fprintf("Near target samples       : %d, speed>%.2f ratio %.1f %%\n", ...
        nnz(near_target), near_target_speed_mps, 100 * near_target_bad_speed_ratio);
else
    fprintf("Near target samples       : 0 below %.2f m\n", near_target_m);
end

fprintf("\n--- Motor context ---\n");
fprintf("PWM mean during hold      : mean %.1f, min %.1f, max %.1f us\n", ...
    mean(pwm_mean(m_inc), "omitnan"), min(pwm_mean(m_inc), [], "omitnan"), ...
    max(pwm_mean(m_inc), [], "omitnan"));
fprintf("PWM span during hold      : mean %.1f, p95 %.1f, max %.1f us\n", ...
    mean(pwm_span(m_inc), "omitnan"), pctl(pwm_span(m_inc), 95), ...
    max(pwm_span(m_inc), [], "omitnan"));
fprintf("Pair diff (M1+M3)-(M2+M4): mean %.1f, min %.1f, max %.1f us\n", ...
    mean((T.M1(m_inc)+T.M3(m_inc))-(T.M2(m_inc)+T.M4(m_inc)), "omitnan"), ...
    min((T.M1(m_inc)+T.M3(m_inc))-(T.M2(m_inc)+T.M4(m_inc)), [], "omitnan"), ...
    max((T.M1(m_inc)+T.M3(m_inc))-(T.M2(m_inc)+T.M4(m_inc)), [], "omitnan"));

fprintf("\n--- Interpretation ---\n");
if pos_sp_drift <= pos_sp_drift_warn_m
    fprintf("[OK] PosHold setpoint was latched and stable.\n");
else
    fprintf("[WARN] PosHold setpoint moved during hold; check latch/reset logic.\n");
end
if max(ekf_gnss_hdiff(m_inc), [], "omitnan") <= ekf_gnss_warn_m
    fprintf("[OK] EKF horizontal position and GNSS position agree closely in this log.\n");
else
    fprintf("[WARN] EKF-GNSS mismatch is large; estimator/GNSS quality needs review.\n");
end
fprintf("[OBS] The first large motion is body-left: peak %.2f m at %.2f s.\n", peak_left, t(i_peak_left));
fprintf("[OBS] The controller later recovered near the setpoint: best late err %.2f m at %.2f s.\n", ...
    best_err_late, t(i_best_late));
if body_left_disp(i1) < -0.5
    fprintf("[OBS] After recovery, the vehicle overshot to the opposite side by %.2f m.\n", -body_left_disp(i1));
end
if max(yaw_abs(m_inc), [], "omitnan") > yaw_very_bad_deg
    fprintf("[LIKELY] Yaw control/authority is a major contributor: yaw error reached %.1f deg.\n", ...
        max(yaw_abs(m_inc), [], "omitnan"));
end
if near_target_bad_speed_ratio > 0.3
    fprintf("[LIKELY] Horizontal velocity damping is insufficient near the setpoint.\n");
end
fprintf("[NEXT LOG] Add actual body rates, rate outputs, motor saturation flags, GNSS hAcc/sAcc, and EKF std.\n");

%% Figures
figs = gobjects(0);
fig_names = strings(0);

fig1 = figure("Name", "01 PosHold trajectory and NE position", "Color", "w");
figs(end+1) = fig1; fig_names(end+1) = "01_poshold_trajectory.png";
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(pos_e, pos_n, "b", "LineWidth", 1.2); hold on;
plot(gnss_e, gnss_n, "Color", [0.4 0.4 0.4], "LineWidth", 0.9);
plot(pos_e(i0), pos_n(i0), "go", "MarkerFaceColor", "g");
plot(pos_e(i_peak_left), pos_n(i_peak_left), "ro", "MarkerFaceColor", "r");
plot(pos_e(i_best_late), pos_n(i_best_late), "co", "MarkerFaceColor", "c");
plot(T.pos_sp_e_m(i0:i1), T.pos_sp_n_m(i0:i1), "k--", "LineWidth", 1.1);
quiver(pos_e(i0), pos_n(i0), body_forward_ne(2), body_forward_ne(1), 0.8, ...
    "Color", [0.0 0.45 0.9], "LineWidth", 1.8, "MaxHeadSize", 1.0);
quiver(pos_e(i0), pos_n(i0), body_left_ne(2), body_left_ne(1), 0.8, ...
    "Color", [0.9 0.0 0.0], "LineWidth", 1.8, "MaxHeadSize", 1.0);
axis equal; grid on; xlabel("East m"); ylabel("North m");
title("Horizontal trajectory. Blue arrow=body F, red arrow=body left at latch.");
legend("EKF", "GNSS", "latch", "peak left", "best recovery", "SP", ...
    "body F", "body left", "Location", "best");

nexttile;
plot(t, pos_n, "b", "LineWidth", 1.1); hold on;
plot(t, pos_e, "r", "LineWidth", 1.1);
plot(t, T.pos_sp_n_m, "b--", "LineWidth", 1.0);
plot(t, T.pos_sp_e_m, "r--", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; xlabel("time s"); ylabel("m");
title("N/E position and fixed PosHold setpoint");
legend("pos N", "pos E", "sp N", "sp E", "PosHold", "Location", "best");

fig2 = figure("Name", "02 Body-frame drift recovery", "Color", "w");
figs(end+1) = fig2; fig_names(end+1) = "02_body_frame_drift_recovery.png";
tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, body_left_disp, "r", "LineWidth", 1.2); hold on;
plot(t, body_forward_disp, "b", "LineWidth", 1.1);
xline(t(i_peak_left), "r--", "peak left");
xline(t(i_best_late), "c--", "best recovery");
yline(0, "k:");
shade_mask(t, m_inc);
grid on; ylabel("m");
title("Displacement projected using latch yaw");
legend("body-left", "body-forward", "Location", "best");

nexttile;
plot(t, pos_err_mag, "k", "LineWidth", 1.2); hold on;
plot(t, pos_err_n, "b", "LineWidth", 1.0);
plot(t, pos_err_e, "r", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; ylabel("m");
title("Position error");
legend("|pos err|", "err N", "err E", "Location", "best");

nexttile;
plot(t, vel_mag, "k", "LineWidth", 1.2); hold on;
plot(t, vel_n, "b", "LineWidth", 1.0);
plot(t, vel_e, "r", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; xlabel("time s"); ylabel("m/s");
title("Filtered horizontal velocity");
legend("|vel|", "vel N", "vel E", "Location", "best");

fig3 = figure("Name", "03 Yaw and attitude tracking", "Color", "w");
figs(end+1) = fig3; fig_names(end+1) = "03_yaw_attitude_tracking.png";
tiledlayout(4, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, yaw, "k", "LineWidth", 1.1); hold on;
plot(t, yaw_des, "k--", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; ylabel("deg");
title("Yaw actual vs desired");
legend("yaw", "yaw des", "Location", "best");

nexttile;
plot(t, yaw_err, "m", "LineWidth", 1.1); hold on;
plot(t, yaw_rate_sp, "Color", [0.2 0.2 0.8], "LineWidth", 1.0);
yline(yaw_bad_deg, "r:");
yline(-yaw_bad_deg, "r:");
shade_mask(t, m_inc);
grid on; ylabel("deg, deg/s");
title("Yaw error and yaw-rate setpoint");
legend("yaw err", "yaw rate sp", "Location", "best");

nexttile;
plot(t, roll, "b", "LineWidth", 1.0); hold on;
plot(t, roll_des, "b--", "LineWidth", 1.0);
plot(t, pitch, "r", "LineWidth", 1.0);
plot(t, pitch_des, "r--", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; ylabel("deg");
title("Roll/pitch tracking");
legend("roll", "roll des", "pitch", "pitch des", "Location", "best");

nexttile;
plot(t, roll_track_err, "b", "LineWidth", 1.0); hold on;
plot(t, pitch_track_err, "r", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; xlabel("time s"); ylabel("deg");
title("Attitude tracking error: desired - actual");
legend("roll err", "pitch err", "Location", "best");

fig4 = figure("Name", "04 Position velocity command chain", "Color", "w");
figs(end+1) = fig4; fig_names(end+1) = "04_pos_vel_command_chain.png";
tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, pos_err_n, "b", "LineWidth", 1.0); hold on;
plot(t, pos_err_e, "r", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; ylabel("m");
title("Position error into position controller");
legend("err N", "err E", "Location", "best");

nexttile;
plot(t, vel_sp_n, "b--", "LineWidth", 1.0); hold on;
plot(t, vel_sp_e, "r--", "LineWidth", 1.0);
plot(t, vel_n, "b", "LineWidth", 1.0);
plot(t, vel_e, "r", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; ylabel("m/s");
title("Velocity setpoint vs filtered velocity");
legend("sp N", "sp E", "vel N", "vel E", "Location", "best");

nexttile;
plot(t, roll_des, "b", "LineWidth", 1.0); hold on;
plot(t, pitch_des, "r", "LineWidth", 1.0);
shade_mask(t, m_inc);
grid on; xlabel("time s"); ylabel("deg");
title("Final attitude command from horizontal controller");
legend("roll des slew", "pitch des slew", "Location", "best");

fig5 = figure("Name", "05 Motor and altitude context", "Color", "w");
figs(end+1) = fig5; fig_names(end+1) = "05_motor_altitude_context.png";
tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, M, "LineWidth", 1.0); hold on;
shade_mask(t, m_inc);
grid on; ylabel("us");
title("Motor PWM");
legend("M1", "M2", "M3", "M4", "PosHold", "Location", "best");

nexttile;
plot(t, pwm_mean, "k", "LineWidth", 1.1); hold on;
plot(t, pwm_span, "Color", [0.9 0.3 0.0], "LineWidth", 1.1);
shade_mask(t, m_inc);
grid on; ylabel("us");
title("PWM mean and span");
legend("mean", "span", "Location", "best");

nexttile;
if ismember("alt_meas_m", names)
    plot(t, T.alt_meas_m, "b", "LineWidth", 1.0); hold on;
end
if ismember("alt_sp_m", names)
    plot(t, T.alt_sp_m, "b--", "LineWidth", 1.0);
end
if ismember("alt_cmd_pwm_us", names)
    yyaxis right;
    plot(t, T.alt_cmd_pwm_us, "Color", [0.8 0.2 0.0], "LineWidth", 1.0);
    ylabel("alt cmd pwm us");
    yyaxis left;
end
shade_mask(t, m_inc);
grid on; xlabel("time s"); ylabel("m");
title("Altitude context");
legend("alt meas", "alt sp", "alt cmd pwm", "Location", "best");

if save_figures
    if ~isfolder(out_dir)
        mkdir(out_dir);
    end
    for k = 1:numel(figs)
        exportgraphics(figs(k), fullfile(out_dir, fig_names(k)), "Resolution", 180);
    end
    fprintf("\nSaved figures to: %s\n", out_dir);
end

%% Local functions
function x = get_col(T, name, default_value)
    if ismember(name, string(T.Properties.VariableNames))
        x = T.(name);
    else
        x = default_value;
    end
end

function m = get_logical(T, name)
    if ismember(name, string(T.Properties.VariableNames))
        m = logical(T.(name));
    else
        m = false(height(T), 1);
    end
end

function segs = mask_segments(mask)
    mask = logical(mask(:));
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
    segs = [starts, ends];
end

function y = pctl(x, pct)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = prctile(x, pct);
    end
end

function y = rms_omitnan(x)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = sqrt(mean(x.^2));
    end
end

function s = pass_fail(tf)
    if tf
        s = "OK";
    else
        s = "WARN";
    end
end

function idx = first_cross_after(x, i_start, i_end, threshold, mode)
    idx = NaN;
    if i_start > i_end
        return;
    end
    switch mode
        case "below"
            hit = find(x(i_start:i_end) <= threshold, 1, "first");
        case "above"
            hit = find(x(i_start:i_end) >= threshold, 1, "first");
        otherwise
            error("Unknown crossing mode.");
    end
    if ~isempty(hit)
        idx = i_start + hit - 1;
    end
end

function print_cross(label, idx, t, body_left_disp, pos_err_mag)
    if isnan(idx)
        fprintf("%-28s: never\n", label);
    else
        fprintf("%-28s: %.3f s, body-left %.3f m, err %.3f m\n", ...
            label, t(idx), body_left_disp(idx), pos_err_mag(idx));
    end
end

function shade_mask(t, mask)
    mask = logical(mask(:));
    if ~any(mask)
        return;
    end
    ax = gca;
    yl = ylim(ax);
    segs = mask_segments(mask);
    hold(ax, "on");
    for ii = 1:size(segs, 1)
        xs = [t(segs(ii, 1)), t(segs(ii, 2)), t(segs(ii, 2)), t(segs(ii, 1))];
        ys = [yl(1), yl(1), yl(2), yl(2)];
        p = patch(ax, xs, ys, [0.93 0.93 0.93], ...
            "EdgeColor", "none", "FaceAlpha", 0.35, ...
            "HandleVisibility", "off", "HitTest", "off");
        % uistack can fail on yyaxis axes in some MATLAB versions.
        % Reorder only when MATLAB accepts the current Children list.
        try
            uistack(p, "bottom");
        catch
            try
                ch = ax.Children;
                ax.Children = [ch(ch ~= p); p];
            catch
                % Leave the shade above the data if the axes refuses child
                % reordering. The alpha is low enough to keep plots readable.
            end
        end
    end
    ylim(ax, yl);
end
