clear; clc; close all;

%% STRIX FC EKF Health Check
% Checks EKF alone: velocity, position, covariance, attitude, dt, ZUPT.
% No output files are saved.

this_dir = fileparts(mfilename("fullpath"));
addpath(fullfile(this_dir, "common"));

%% User settings
csv_file = "";              % leave empty to open file picker
cfg = struct();
cfg.analysis_start_sec = 0.0;
cfg.analysis_end_sec = inf;
cfg.use_ekf_ready = true;
cfg.use_gnss_ref_ready = false;
cfg.use_gnss_valid = false;
cfg.title_prefix = "03 EKF Health Check";

ekf_speed_p95_good = 0.20;    % m/s, for ground/static logs
ekf_speed_p95_warn = 0.50;    % m/s
dt_p95_warn = 0.08;           % sec, tune for your log rate
cov_vel_h_warn = 1.0;         % sqrt covariance scale, tune later

%% Load
log = load_vtol_log(csv_file, cfg);
T = log.T; cols = log.cols; N = log.N; t = log.t; mask = log.mask;

[ekf_speed_h, speed_source] = strix_ekf_speed_h(T, cols, N);
ekf_vel_n = strix_col(T, cols, "ekf_vel_n", nan(N, 1));
ekf_vel_e = strix_col(T, cols, "ekf_vel_e", nan(N, 1));
ekf_vel_d = strix_col(T, cols, "ekf_vel_d", nan(N, 1));

ekf_pos_n = strix_col(T, cols, "ekf_pos_n", nan(N, 1));
ekf_pos_e = strix_col(T, cols, "ekf_pos_e", nan(N, 1));
ekf_pos_d = strix_col(T, cols, "ekf_pos_d", nan(N, 1));

roll_deg = strix_col(T, cols, "ekf_roll_deg", strix_col(T, cols, "roll_deg", nan(N, 1)));
pitch_deg = strix_col(T, cols, "ekf_pitch_deg", strix_col(T, cols, "pitch_deg", nan(N, 1)));
yaw_deg = strix_col(T, cols, "ekf_yaw_deg", strix_col(T, cols, "yaw_deg", nan(N, 1)));

p_cov_n = strix_col(T, cols, "ekf_p_cov_n", nan(N, 1));
p_cov_e = strix_col(T, cols, "ekf_p_cov_e", nan(N, 1));
p_cov_d = strix_col(T, cols, "ekf_p_cov_d", nan(N, 1));
v_cov_n = strix_col(T, cols, "ekf_v_cov_n", nan(N, 1));
v_cov_e = strix_col(T, cols, "ekf_v_cov_e", nan(N, 1));
v_cov_d = strix_col(T, cols, "ekf_v_cov_d", nan(N, 1));

sqrt_p_cov_h = sqrt(max(0, p_cov_n + p_cov_e));
sqrt_v_cov_h = sqrt(max(0, v_cov_n + v_cov_e));

ekf_dt = strix_col(T, cols, "ekf_dt", [NaN; diff(t)]);
ekf_dt_clamped = strix_boolcol(T, cols, "ekf_dt_clamped", false(N, 1));
ekf_ready = strix_boolcol(T, cols, "ekf_ready", false(N, 1));

zupt_active = strix_boolcol(T, cols, "zupt_active", false(N, 1));
stationary_detected = strix_boolcol(T, cols, "stationary_detected", false(N, 1));
ekf_zupt_applied = strix_boolcol(T, cols, "ekf_zupt_applied", false(N, 1));

fprintf("EKF speed source: %s\n\n", speed_source);

fprintf("=================================================\n");
fprintf("EKF Health Summary\n");
fprintf("=================================================\n");
fprintf("ekf_ready samples                  : %d\n", sum(mask & ekf_ready));
strix_print_metric("ekf_speed_h", ekf_speed_h(mask), "m/s");
strix_print_metric("ekf_vel_d", ekf_vel_d(mask), "m/s");
strix_print_metric("ekf_pos_n", ekf_pos_n(mask), "m");
strix_print_metric("ekf_pos_e", ekf_pos_e(mask), "m");
strix_print_metric("ekf_pos_d", ekf_pos_d(mask), "m");
strix_print_metric("roll_deg", roll_deg(mask), "deg");
strix_print_metric("pitch_deg", pitch_deg(mask), "deg");
strix_print_metric("yaw_deg", yaw_deg(mask), "deg");
strix_print_metric("sqrt(P_vel_H)", sqrt_v_cov_h(mask), "m/s");
strix_print_metric("sqrt(P_pos_H)", sqrt_p_cov_h(mask), "m");
strix_print_metric("ekf_dt", ekf_dt(mask), "s");
fprintf("dt_clamped samples                 : %d\n", sum(mask & ekf_dt_clamped));
fprintf("stationary_detected samples        : %d\n", sum(mask & stationary_detected));
fprintf("zupt_active samples                : %d\n", sum(mask & zupt_active));
fprintf("ekf_zupt_applied samples           : %d\n", sum(mask & ekf_zupt_applied));

speed_p95 = strix_pctf(ekf_speed_h(mask), 95);
dt_p95 = strix_pctf(ekf_dt(mask), 95);
cov_vh_p95 = strix_pctf(sqrt_v_cov_h(mask), 95);

ekf_pass = speed_p95 < ekf_speed_p95_good && sum(mask & ekf_dt_clamped) == 0;
ekf_warn = speed_p95 < ekf_speed_p95_warn && dt_p95 < dt_p95_warn && cov_vh_p95 < cov_vel_h_warn;

fprintf("-------------------------------------------------\n");
fprintf("EKF status                         : %s\n", strix_status(ekf_pass, ekf_warn));
fprintf("=================================================\n\n");

%% Plots
figure("Name", "03 EKF Health - Velocity Position", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, ekf_vel_n, "LineWidth", 1.0); hold on;
plot(t, ekf_vel_e, "LineWidth", 1.0);
plot(t, ekf_vel_d, "LineWidth", 1.0);
grid on; ylabel("vel [m/s]");
legend("N", "E", "D", "Location", "best");
title("EKF velocity");

nexttile;
plot(t, ekf_speed_h, "LineWidth", 1.0);
grid on; ylabel("speed H [m/s]");
title("EKF horizontal speed");

nexttile;
plot(t, ekf_pos_n, "LineWidth", 1.0); hold on;
plot(t, ekf_pos_e, "LineWidth", 1.0);
plot(t, ekf_pos_d, "LineWidth", 1.0);
grid on; ylabel("pos [m]");
legend("N", "E", "D", "Location", "best");
title("EKF position");

nexttile;
plot(t, ekf_dt, "LineWidth", 1.0); hold on;
stem(t(ekf_dt_clamped), double(ekf_dt_clamped(ekf_dt_clamped)) * strix_maxf(ekf_dt), "filled");
grid on; xlabel("time [s]"); ylabel("dt [s]");
title("EKF dt and dt clamped");

figure("Name", "03 EKF Health - Attitude Covariance ZUPT", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, roll_deg, "LineWidth", 1.0); hold on;
plot(t, pitch_deg, "LineWidth", 1.0);
plot(t, yaw_deg, "LineWidth", 1.0);
grid on; ylabel("angle [deg]");
legend("roll", "pitch", "yaw", "Location", "best");
title("EKF attitude");

nexttile;
plot(t, sqrt_v_cov_h, "LineWidth", 1.0); hold on;
plot(t, sqrt(max(0, v_cov_d)), "LineWidth", 1.0);
grid on; ylabel("sqrt P vel");
legend("H", "D", "Location", "best");
title("EKF velocity covariance scale");

nexttile;
plot(t, sqrt_p_cov_h, "LineWidth", 1.0); hold on;
plot(t, sqrt(max(0, p_cov_d)), "LineWidth", 1.0);
grid on; ylabel("sqrt P pos");
legend("H", "D", "Location", "best");
title("EKF position covariance scale");

nexttile;
stairs(t, double(stationary_detected), "LineWidth", 1.0); hold on;
stairs(t, double(zupt_active), "LineWidth", 1.0);
stairs(t, double(ekf_zupt_applied), "LineWidth", 1.0);
grid on; xlabel("time [s]"); ylabel("flag");
legend("stationary", "zupt active", "zupt applied", "Location", "best");
title("ZUPT / stationary flags");
