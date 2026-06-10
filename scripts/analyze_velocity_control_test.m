%% STRIX FC XY Velocity Controller Validation
% 위치제어가 아니라 XY velocity damping(v_N_sp=0, v_E_sp=0) 검증용.
%
% Folder assumption:
%   project_root/
%     data/vel_test_01.CSV
%     scripts/analyze_velocity_control_test.m
%
% This script checks:
%   1) velocity controller ON/OFF timing
%   2) commanded roll/pitch size and saturation
%   3) whether accel command opposes measured velocity
%   4) EKF velocity, GNSS update delta, vibration indicators
%   5) PWM bin summary for hover/velocity-control test

clear; clc; close all;

%% User settings
csv_file = "260610\fc_damp_01.CSV";       % file name inside ../data
analysis_start_sec = 0.0;
analysis_end_sec = inf;

use_ekf_ready = true;
use_gnss_ref_ready = true;

% Thresholds for first low-hover velocity damping test
vel_xy_good_p95 = 0.30;             % [m/s] first target
vel_xy_warn_p95 = 0.60;             % [m/s]
delta_vel_warn_p95 = 0.50;          % [m/s] GNSS correction jump
tilt_warn_deg = 2.5;                % [deg] expected initial limit around 2 deg
tilt_fail_deg = 4.0;                % [deg]
acc_cmd_warn = 0.7;                 % [m/s^2] expected initial clamp around 0.6
braking_good_ratio = 0.70;          % fraction of samples with v dot a_cmd < 0
braking_strong_ratio = 0.85;

pwm_round_step = 50;
min_bin_samples = 10;

write_summary_csv = true;

%% Resolve paths
script_path = mfilename("fullpath");
if strlength(script_path) == 0
    % If run as selected text, fall back to current folder.
    script_dir = pwd;
else
    script_dir = fileparts(script_path);
end

% Expected: script in project_root/scripts, data in project_root/data
project_root = fileparts(script_dir);
data_path = fullfile(project_root, "data", csv_file);

% Fallback: current folder/data
if ~isfile(data_path)
    data_path = fullfile(pwd, "data", csv_file);
end

% Fallback: current folder
if ~isfile(data_path)
    data_path = fullfile(pwd, csv_file);
end

if ~isfile(data_path)
    error("CSV file not found. Tried: ../data/%s, ./data/%s, ./%s", ...
        csv_file, csv_file, csv_file);
end

%% Load
T = readtable(data_path, "VariableNamingRule", "preserve");
cols = string(T.Properties.VariableNames);
N = height(T);

need(cols, "timestamp_ms");

t = double(T.timestamp_ms) * 1e-3;
t = t - t(1);

mask = isfinite(t) & t >= analysis_start_sec & t <= analysis_end_sec;
if use_ekf_ready && ismember("ekf_ready", cols)
    mask = mask & boolcol(T, cols, "ekf_ready", false(N, 1));
end
if use_gnss_ref_ready && ismember("gnss_ref_ready", cols)
    mask = mask & boolcol(T, cols, "gnss_ref_ready", false(N, 1));
end

%% Required velocity-control columns
required_vc = [
    "vel_ctrl_enabled"
    "vel_ctrl_valid"
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
    "vel_acc_cmd_forward_mps2"
    "vel_acc_cmd_right_mps2"
    "vel_roll_des_deg"
    "vel_pitch_des_deg"
];
missing = required_vc(~ismember(required_vc, cols));
if ~isempty(missing)
    error("Missing velocity-control column(s): %s", join(missing, ", "));
end

%% Signals
pwm = load_col(T, cols, "ekf_pwm_mean", nan(N, 1));
if all(~isfinite(pwm)) && all(ismember(["M1","M2","M3","M4"], cols))
    pwm = mean([double(T.M1), double(T.M2), double(T.M3), double(T.M4)], 2, "omitnan");
end

vc_enabled = boolcol(T, cols, "vel_ctrl_enabled", false(N, 1));
vc_valid = boolcol(T, cols, "vel_ctrl_valid", false(N, 1));
vc_acc_sat = boolcol(T, cols, "vel_ctrl_accel_saturated", false(N, 1));
vc_tilt_sat = boolcol(T, cols, "vel_ctrl_tilt_saturated", false(N, 1));

vel_sp_n = load_col(T, cols, "vel_sp_n_mps", nan(N, 1));
vel_sp_e = load_col(T, cols, "vel_sp_e_mps", nan(N, 1));
vel_meas_n = load_col(T, cols, "vel_meas_n_mps", nan(N, 1));
vel_meas_e = load_col(T, cols, "vel_meas_e_mps", nan(N, 1));
vel_filt_n = load_col(T, cols, "vel_filt_n_mps", nan(N, 1));
vel_filt_e = load_col(T, cols, "vel_filt_e_mps", nan(N, 1));
vel_err_n = load_col(T, cols, "vel_err_n_mps", nan(N, 1));
vel_err_e = load_col(T, cols, "vel_err_e_mps", nan(N, 1));

acc_cmd_n = load_col(T, cols, "vel_acc_cmd_n_mps2", nan(N, 1));
acc_cmd_e = load_col(T, cols, "vel_acc_cmd_e_mps2", nan(N, 1));
acc_cmd_forward = load_col(T, cols, "vel_acc_cmd_forward_mps2", nan(N, 1));
acc_cmd_right = load_col(T, cols, "vel_acc_cmd_right_mps2", nan(N, 1));

roll_des = load_col(T, cols, "vel_roll_des_deg", nan(N, 1));
pitch_des = load_col(T, cols, "vel_pitch_des_deg", nan(N, 1));

ekf_vel_n = load_col(T, cols, "ekf_vel_n", nan(N, 1));
ekf_vel_e = load_col(T, cols, "ekf_vel_e", nan(N, 1));
ekf_speed_h = load_col(T, cols, "ekf_speed_h", hypot(ekf_vel_n, ekf_vel_e));

roll_deg = load_col(T, cols, "ekf_roll_deg", load_col(T, cols, "roll_deg", nan(N, 1)));
pitch_deg = load_col(T, cols, "ekf_pitch_deg", load_col(T, cols, "pitch_deg", nan(N, 1)));
yaw_deg = load_col(T, cols, "ekf_yaw_deg", load_col(T, cols, "yaw_deg", nan(N, 1)));

delta_vel_xy = load_col(T, cols, "delta_vel_gnss_update_xy", nan(N, 1));
if all(~isfinite(delta_vel_xy)) && all(ismember(["delta_vel_gnss_update_n","delta_vel_gnss_update_e"], cols))
    delta_vel_xy = hypot(double(T.delta_vel_gnss_update_n), double(T.delta_vel_gnss_update_e));
end

gnss_update = boolcol(T, cols, "gnss_update_executed", false(N, 1));
gnss_accepted = boolcol(T, cols, "gnss_correction_accepted", false(N, 1));

acc_ned_h = load_col(T, cols, "acc_ned_h", nan(N, 1));
raw_acc_norm_g = load_col(T, cols, "raw_acc_norm_g", nan(N, 1));
gyro_lpf_norm = load_col(T, cols, "gyro_lpf_norm", nan(N, 1));

% Derived
vel_meas_xy = hypot(vel_meas_n, vel_meas_e);
vel_filt_xy = hypot(vel_filt_n, vel_filt_e);
vel_sp_xy = hypot(vel_sp_n, vel_sp_e);
vel_err_xy = hypot(vel_err_n, vel_err_e);
acc_cmd_xy = hypot(acc_cmd_n, acc_cmd_e);
tilt_cmd_abs = max(abs(roll_des), abs(pitch_des));

% Core validation mask: use only when controller is actually active and valid
ctrl_mask = mask & vc_enabled & vc_valid;
ctrl_update_mask = ctrl_mask & gnss_update;

% Braking check:
% If v_sp = 0, acc_cmd should oppose measured/filt velocity.
% v dot a_cmd < 0 means damping direction.
brake_dot = vel_filt_n .* acc_cmd_n + vel_filt_e .* acc_cmd_e;
brake_power_like = brake_dot;  % [m^2/s^3], sign only is important here
braking_good = brake_dot < 0;

%% Command window summary
fprintf("\n=================================================\n");
fprintf("STRIX FC XY Velocity Controller Validation\n");
fprintf("=================================================\n");
fprintf("CSV                    : %s\n", data_path);
fprintf("Rows                   : %d\n", N);
fprintf("Valid mask samples     : %d\n", sum(mask));
fprintf("Controller ON samples  : %d\n", sum(mask & vc_enabled));
fprintf("Controller valid samples: %d\n", sum(ctrl_mask));
fprintf("Time range             : %.3f ~ %.3f sec\n", minf(t(mask)), maxf(t(mask)));
fprintf("Control time range     : %.3f ~ %.3f sec\n", minf(t(ctrl_mask)), maxf(t(ctrl_mask)));
fprintf("GNSS update samples    : %d\n", sum(mask & gnss_update));
fprintf("GNSS accepted samples  : %d\n", sum(mask & gnss_accepted));
fprintf("-------------------------------------------------\n");

print_metric("vel_sp_xy", vel_sp_xy(ctrl_mask), "m/s");
print_metric("vel_meas_xy", vel_meas_xy(ctrl_mask), "m/s");
print_metric("vel_filt_xy", vel_filt_xy(ctrl_mask), "m/s");
print_metric("ekf_speed_h", ekf_speed_h(ctrl_mask), "m/s");
print_metric("vel_err_xy", vel_err_xy(ctrl_mask), "m/s");
print_metric("delta_vel_xy on GNSS update", delta_vel_xy(ctrl_update_mask), "m/s");
print_metric("acc_cmd_xy", acc_cmd_xy(ctrl_mask), "m/s^2");
print_metric("roll_des abs", abs(roll_des(ctrl_mask)), "deg");
print_metric("pitch_des abs", abs(pitch_des(ctrl_mask)), "deg");
print_metric("tilt_cmd max(abs roll/pitch)", tilt_cmd_abs(ctrl_mask), "deg");
print_metric("acc_ned_h", acc_ned_h(ctrl_mask), "m/s^2");
print_metric("raw_acc_norm_g", raw_acc_norm_g(ctrl_mask), "g");
print_metric("gyro_lpf_norm", gyro_lpf_norm(ctrl_mask), "rad/s");

fprintf("-------------------------------------------------\n");
fprintf("accel saturation samples: %d / %d (%.2f %%)\n", ...
    sum(ctrl_mask & vc_acc_sat), sum(ctrl_mask), ratio_percent(sum(ctrl_mask & vc_acc_sat), sum(ctrl_mask)));
fprintf("tilt saturation samples : %d / %d (%.2f %%)\n", ...
    sum(ctrl_mask & vc_tilt_sat), sum(ctrl_mask), ratio_percent(sum(ctrl_mask & vc_tilt_sat), sum(ctrl_mask)));

brake_ratio = ratio_percent(sum(ctrl_mask & braking_good), sum(ctrl_mask));
fprintf("braking ratio, v dot a_cmd < 0 : %.2f %%\n", brake_ratio);
fprintf("mean(v dot a_cmd)              : %.6f\n", meanf(brake_power_like(ctrl_mask)));
fprintf("corr(vel_filt_n, acc_cmd_n)    : %.4f\n", corrf(vel_filt_n(ctrl_mask), acc_cmd_n(ctrl_mask)));
fprintf("corr(vel_filt_e, acc_cmd_e)    : %.4f\n", corrf(vel_filt_e(ctrl_mask), acc_cmd_e(ctrl_mask)));
fprintf("corr(acc_cmd_xy, tilt_cmd_abs) : %.4f\n", corrf(acc_cmd_xy(ctrl_mask), tilt_cmd_abs(ctrl_mask)));
fprintf("corr(acc_ned_h, ekf_speed_h)   : %.4f\n", corrf(acc_ned_h(ctrl_mask), ekf_speed_h(ctrl_mask)));

fprintf("-------------------------------------------------\n");
vel_p95 = pctf(ekf_speed_h(ctrl_mask), 95);
delta_p95 = pctf(delta_vel_xy(ctrl_update_mask), 95);
tilt_p95 = pctf(tilt_cmd_abs(ctrl_mask), 95);
acc_cmd_p95 = pctf(acc_cmd_xy(ctrl_mask), 95);

fprintf("Velocity hold status : %s\n", status3(vel_p95 <= vel_xy_good_p95, vel_p95 <= vel_xy_warn_p95));
fprintf("GNSS correction jump : %s\n", status3(delta_p95 <= delta_vel_warn_p95, delta_p95 <= delta_vel_warn_p95 * 1.5));
fprintf("Tilt command size    : %s\n", status3(tilt_p95 <= tilt_warn_deg, tilt_p95 <= tilt_fail_deg));
fprintf("Accel command size   : %s\n", status3(acc_cmd_p95 <= acc_cmd_warn, acc_cmd_p95 <= acc_cmd_warn * 1.5));
fprintf("Damping sign check   : %s\n", status3(brake_ratio >= 100*braking_strong_ratio, brake_ratio >= 100*braking_good_ratio));
fprintf("=================================================\n\n");

%% PWM bin summary
pwm_round = round(pwm ./ pwm_round_step) .* pwm_round_step;
pwm_round(~isfinite(pwm_round)) = NaN;
labels = unique(pwm_round(ctrl_mask & isfinite(pwm_round)));
bin_summary = table();

for i = 1:numel(labels)
    idx = ctrl_mask & pwm_round == labels(i);
    if sum(idx) < min_bin_samples
        continue;
    end
    upd = idx & gnss_update;

    row = table();
    row.pwm_label = labels(i);
    row.samples = sum(idx);
    row.duration_approx = maxf(t(idx)) - minf(t(idx));
    row.pwm_mean = meanf(pwm(idx));
    row.vel_xy_p95 = pctf(ekf_speed_h(idx), 95);
    row.vel_filt_xy_p95 = pctf(vel_filt_xy(idx), 95);
    row.delta_update_count = sum(upd);
    row.delta_vel_xy_p95 = pctf(delta_vel_xy(upd), 95);
    row.acc_cmd_xy_p95 = pctf(acc_cmd_xy(idx), 95);
    row.tilt_cmd_p95_deg = pctf(tilt_cmd_abs(idx), 95);
    row.roll_des_p95_deg = pctf(abs(roll_des(idx)), 95);
    row.pitch_des_p95_deg = pctf(abs(pitch_des(idx)), 95);
    row.accel_sat_pct = ratio_percent(sum(idx & vc_acc_sat), sum(idx));
    row.tilt_sat_pct = ratio_percent(sum(idx & vc_tilt_sat), sum(idx));
    row.braking_ratio_pct = ratio_percent(sum(idx & braking_good), sum(idx));
    row.mean_v_dot_acc = meanf(brake_dot(idx));
    row.acc_ned_h_p95 = pctf(acc_ned_h(idx), 95);
    row.raw_acc_norm_p95_g = pctf(raw_acc_norm_g(idx), 95);
    row.raw_acc_norm_std_g = stdf(raw_acc_norm_g(idx));
    row.gyro_lpf_p95 = pctf(gyro_lpf_norm(idx), 95);

    bin_summary = [bin_summary; row]; %#ok<AGROW>
end

fprintf("=================================================\n");
fprintf("PWM Bin Summary During Velocity Control\n");
fprintf("=================================================\n");
disp(bin_summary);

%% Worst events
fprintf("=================================================\n");
fprintf("Worst Events During Velocity Control\n");
fprintf("=================================================\n");
print_top_events("ekf_speed_h", ekf_speed_h, ctrl_mask, 12, ...
    t, pwm, ekf_speed_h, vel_filt_xy, acc_cmd_xy, tilt_cmd_abs, ...
    delta_vel_xy, acc_ned_h, raw_acc_norm_g, gyro_lpf_norm, brake_dot, gnss_update);

print_top_events("bad braking, v dot a_cmd positive", brake_dot, ctrl_mask & brake_dot > 0, 12, ...
    t, pwm, ekf_speed_h, vel_filt_xy, acc_cmd_xy, tilt_cmd_abs, ...
    delta_vel_xy, acc_ned_h, raw_acc_norm_g, gyro_lpf_norm, brake_dot, gnss_update);

print_top_events("delta_vel_xy", delta_vel_xy, ctrl_update_mask, 12, ...
    t, pwm, ekf_speed_h, vel_filt_xy, acc_cmd_xy, tilt_cmd_abs, ...
    delta_vel_xy, acc_ned_h, raw_acc_norm_g, gyro_lpf_norm, brake_dot, gnss_update);

fprintf("=================================================\n\n");

%% Save summary
if write_summary_csv
    results_dir = fullfile(project_root, "results");
    if ~exist(results_dir, "dir")
        mkdir(results_dir);
    end
    [~, base_name, ~] = fileparts(data_path);
    out_path = fullfile(results_dir, base_name + "_velocity_control_summary.csv");
    writetable(bin_summary, out_path);
    fprintf("Saved velocity control summary: %s\n", out_path);
end

%% Figure 1: time series
figure("Name", "Velocity control time series", "Color", "w");
tiledlayout(7, 1, "TileSpacing", "compact");

nexttile;
plot(t, pwm, "LineWidth", 1.0); grid on;
ylabel("PWM [us]");
title("PWM");

nexttile;
plot(t, double(vc_enabled), "LineWidth", 1.0); hold on;
plot(t, double(vc_valid), "LineWidth", 1.0);
plot(t, double(vc_acc_sat), "--", "LineWidth", 0.8);
plot(t, double(vc_tilt_sat), "--", "LineWidth", 0.8);
grid on; ylabel("flag");
legend("enabled", "valid", "acc sat", "tilt sat", "Location", "best");
title("Velocity controller flags");

nexttile;
plot(t, vel_meas_n, "LineWidth", 0.8); hold on;
plot(t, vel_filt_n, "LineWidth", 1.0);
plot(t, vel_sp_n, "--", "LineWidth", 1.0);
grid on; ylabel("N vel [m/s]");
legend("meas", "filt", "sp", "Location", "best");
title("North velocity");

nexttile;
plot(t, vel_meas_e, "LineWidth", 0.8); hold on;
plot(t, vel_filt_e, "LineWidth", 1.0);
plot(t, vel_sp_e, "--", "LineWidth", 1.0);
grid on; ylabel("E vel [m/s]");
legend("meas", "filt", "sp", "Location", "best");
title("East velocity");

nexttile;
plot(t, ekf_speed_h, "LineWidth", 1.0); hold on;
yline(vel_xy_good_p95, "--", "good");
yline(vel_xy_warn_p95, "--r", "warn");
grid on; ylabel("speed H [m/s]");
title("Horizontal speed");

nexttile;
plot(t, roll_des, "LineWidth", 1.0); hold on;
plot(t, pitch_des, "LineWidth", 1.0);
yline(tilt_warn_deg, "--r", "+warn");
yline(-tilt_warn_deg, "--r", "-warn");
grid on; ylabel("cmd [deg]");
legend("roll des", "pitch des", "Location", "best");
title("Velocity-controller roll/pitch setpoint");

nexttile;
plot(t, acc_cmd_n, "LineWidth", 1.0); hold on;
plot(t, acc_cmd_e, "LineWidth", 1.0);
plot(t, brake_dot, "LineWidth", 0.8);
yline(0, "--");
grid on; xlabel("time [s]"); ylabel("cmd / dot");
legend("acc N", "acc E", "v dot acc", "Location", "best");
title("Acceleration command and braking sign");

%% Figure 2: command relationship
figure("Name", "Velocity control command validation", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");

idx = ctrl_mask & isfinite(pwm);

nexttile;
scatter(vel_filt_n(idx), acc_cmd_n(idx), 12, pwm(idx), "filled");
grid on; xlabel("vel filt N [m/s]"); ylabel("acc cmd N [m/s^2]");
title("N: velocity vs accel command"); colorbar;
xline(0, "--"); yline(0, "--");

nexttile;
scatter(vel_filt_e(idx), acc_cmd_e(idx), 12, pwm(idx), "filled");
grid on; xlabel("vel filt E [m/s]"); ylabel("acc cmd E [m/s^2]");
title("E: velocity vs accel command"); colorbar;
xline(0, "--"); yline(0, "--");

nexttile;
scatter(vel_filt_xy(idx), acc_cmd_xy(idx), 12, pwm(idx), "filled");
grid on; xlabel("filtered speed H [m/s]"); ylabel("acc cmd H [m/s^2]");
title("speed vs accel command magnitude"); colorbar;

nexttile;
scatter(brake_dot(idx), ekf_speed_h(idx), 12, pwm(idx), "filled");
grid on; xlabel("v dot acc_cmd  (<0 is braking)"); ylabel("ekf speed H [m/s]");
title("Braking sign check"); colorbar;
xline(0, "--r");

%% Figure 3: controller output and attitude tracking
figure("Name", "Velocity controller output and attitude", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");

nexttile;
plot(t, roll_des, "LineWidth", 1.0); hold on;
plot(t, roll_deg, "LineWidth", 0.8);
grid on; xlabel("time [s]"); ylabel("roll [deg]");
legend("roll des", "roll", "Location", "best");
title("Roll command vs attitude");

nexttile;
plot(t, pitch_des, "LineWidth", 1.0); hold on;
plot(t, pitch_deg, "LineWidth", 0.8);
grid on; xlabel("time [s]"); ylabel("pitch [deg]");
legend("pitch des", "pitch", "Location", "best");
title("Pitch command vs attitude");

nexttile;
scatter(roll_des(idx), roll_deg(idx), 12, pwm(idx), "filled");
grid on; xlabel("roll des [deg]"); ylabel("roll [deg]");
title("Roll tracking scatter"); colorbar;
xline(0, "--"); yline(0, "--");

nexttile;
scatter(pitch_des(idx), pitch_deg(idx), 12, pwm(idx), "filled");
grid on; xlabel("pitch des [deg]"); ylabel("pitch [deg]");
title("Pitch tracking scatter"); colorbar;
xline(0, "--"); yline(0, "--");

%% Figure 4: vibration influence during velocity control
figure("Name", "Velocity control with vibration indicators", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");

nexttile;
scatter(acc_ned_h(idx), ekf_speed_h(idx), 12, pwm(idx), "filled");
grid on; xlabel("acc NED H [m/s^2]"); ylabel("ekf speed H [m/s]");
title("acc_ned_h vs speed"); colorbar;

nexttile;
scatter(raw_acc_norm_g(idx), acc_ned_h(idx), 12, pwm(idx), "filled");
grid on; xlabel("raw acc norm [g]"); ylabel("acc NED H [m/s^2]");
title("raw acc norm vs rotated acc"); colorbar;

nexttile;
scatter(gyro_lpf_norm(idx), ekf_speed_h(idx), 12, pwm(idx), "filled");
grid on; xlabel("gyro LPF norm [rad/s]"); ylabel("ekf speed H [m/s]");
title("gyro vs speed"); colorbar;

nexttile;
scatter(delta_vel_xy(ctrl_update_mask), ekf_speed_h(ctrl_update_mask), 12, pwm(ctrl_update_mask), "filled");
grid on; xlabel("GNSS delta vel XY [m/s]"); ylabel("ekf speed H [m/s]");
title("GNSS correction jump vs speed"); colorbar;

%% Local functions
function need(cols, names)
    missing = string.empty;
    for i = 1:numel(names)
        if ~ismember(names(i), cols)
            missing(end + 1) = names(i); %#ok<AGROW>
        end
    end
    if ~isempty(missing)
        error("Missing required column(s): %s", join(missing, ", "));
    end
end

function y = load_col(T, cols, name, default_value)
    if ismember(name, cols)
        y = double(T.(name));
    else
        y = default_value;
    end
end

function y = boolcol(T, cols, name, default_value)
    if ismember(name, cols)
        y = double(T.(name)) ~= 0;
    else
        y = default_value;
    end
end

function y = finite_vec(x)
    y = x(isfinite(x));
end

function y = meanf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = mean(x); end
end

function y = minf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = min(x); end
end

function y = maxf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = max(x); end
end

function y = rmsf(x)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = sqrt(mean(x.^2)); end
end

function y = stdf(x)
    x = finite_vec(x);
    if numel(x) < 2, y = NaN; else, y = std(x); end
end

function y = pctf(x, p)
    x = finite_vec(x);
    if isempty(x), y = NaN; else, y = prctile(x, p); end
end

function y = corrf(a, b)
    idx = isfinite(a) & isfinite(b);
    if sum(idx) < 3
        y = NaN;
    else
        C = corrcoef(a(idx), b(idx));
        y = C(1, 2);
    end
end

function p = ratio_percent(num, den)
    if den <= 0
        p = NaN;
    else
        p = 100.0 * double(num) / double(den);
    end
end

function s = status3(pass_ok, warn_ok)
    if pass_ok
        s = "PASS";
    elseif warn_ok
        s = "WARN";
    else
        s = "FAIL";
    end
end

function print_metric(name, x, unit)
    fprintf("%-34s RMS=%8.4f  P95=%8.4f  MAX=%8.4f  MEAN=%8.4f  [%s]\n", ...
        name, rmsf(x), pctf(x, 95), maxf(x), meanf(x), unit);
end

function print_top_events(name, score, event_mask, top_n, ...
                          t, pwm, ekf_speed_h, vel_filt_xy, acc_cmd_xy, tilt_cmd_abs, ...
                          delta_vel_xy, acc_ned_h, raw_acc_norm_g, gyro_lpf_norm, brake_dot, gnss_update)
    idx = find(event_mask & isfinite(score));
    if isempty(idx)
        fprintf("%s top events: none\n\n", name);
        return;
    end

    [~, order] = sort(score(idx), "descend");
    idx = idx(order(1:min(top_n, numel(order))));

    fprintf("%s top events:\n", name);
    fprintf("  rank   t[s]      PWM    speedH  vFiltH  accCmdH  tiltCmd  dV_gnss  accNEDH  rawAccG  gyroLPF  vDotAcc  upd\n");
    for k = 1:numel(idx)
        ii = idx(k);
        fprintf("  %4d %8.3f %8.1f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %9.5f  %3d\n", ...
            k, t(ii), pwm(ii), ekf_speed_h(ii), vel_filt_xy(ii), acc_cmd_xy(ii), tilt_cmd_abs(ii), ...
            delta_vel_xy(ii), acc_ned_h(ii), raw_acc_norm_g(ii), gyro_lpf_norm(ii), brake_dot(ii), ...
            gnss_update(ii));
    end
    fprintf("\n");
end
