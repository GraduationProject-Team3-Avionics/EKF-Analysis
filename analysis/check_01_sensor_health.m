clear; clc; close all;

%% STRIX FC Sensor Health Check
% Checks IMU / barometer / PWM coupling only.
% No CSV, MAT, PNG, or FIG files are saved.

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
cfg.title_prefix = "01 Sensor Health Check";

raw_acc_std_warn_g = 0.10;
raw_acc_std_fail_g = 0.30;
acc_ned_h_warn = 1.0;       % m/s^2
acc_ned_h_fail = 3.0;       % m/s^2
gyro_p95_warn = 0.35;       % rad/s
gyro_p95_fail = 0.60;       % rad/s
baro_step_warn = 0.15;      % m/sample, tune after seeing logs

%% Load
log = load_vtol_log(csv_file, cfg);
T = log.T; cols = log.cols; N = log.N; t = log.t; mask = log.mask;

[pwm, pwm_source] = strix_pwm(T, cols, N);
[raw_acc_norm_g, raw_acc_source] = strix_raw_acc_norm_g(T, cols, N);
[acc_ned_h, acc_ned_source] = strix_acc_ned_h(T, cols, N);
[gyro_raw_norm, gyro_lpf_norm, gyro_corrected_norm, gyro_source] = strix_gyro_norms(T, cols, N);

baro_rel_alt = strix_col(T, cols, "baro_rel_alt_m", nan(N, 1));
baro_raw_pa = strix_col(T, cols, "baro_raw_pa", nan(N, 1));
baro_age_ms = strix_col(T, cols, "baro_age_ms", nan(N, 1));
baro_updated = strix_boolcol(T, cols, "baro_updated", false(N, 1));

baro_step = [NaN; abs(diff(baro_rel_alt))];

%% Console summary
fprintf("Signal sources\n");
fprintf("  PWM          : %s\n", pwm_source);
fprintf("  Raw accel    : %s\n", raw_acc_source);
fprintf("  acc_ned_h    : %s\n", acc_ned_source);
fprintf("  Gyro         : %s\n", gyro_source);
fprintf("\n");

fprintf("=================================================\n");
fprintf("Sensor Health Summary\n");
fprintf("=================================================\n");
strix_print_metric("raw_acc_norm_g", raw_acc_norm_g(mask), "g");
fprintf("%-34s STD =%9.4f  [g]\n", "raw_acc_norm_g", strix_stdf(raw_acc_norm_g(mask)));
strix_print_metric("acc_ned_h", acc_ned_h(mask), "m/s^2");
strix_print_metric("gyro_raw_norm", gyro_raw_norm(mask), "rad/s");
strix_print_metric("gyro_lpf_norm", gyro_lpf_norm(mask), "rad/s");
strix_print_metric("gyro_corrected_norm", gyro_corrected_norm(mask), "rad/s");
strix_print_metric("baro_rel_alt_m", baro_rel_alt(mask), "m");
strix_print_metric("abs(diff(baro_rel_alt_m))", baro_step(mask), "m/sample");
strix_print_metric("baro_age_ms", baro_age_ms(mask), "ms");
fprintf("Baro updated samples                : %d\n", sum(mask & baro_updated));
fprintf("corr(PWM, raw_acc_norm_g)           : %.4f\n", strix_corrf(pwm(mask), raw_acc_norm_g(mask)));
fprintf("corr(PWM, acc_ned_h)                : %.4f\n", strix_corrf(pwm(mask), acc_ned_h(mask)));
fprintf("corr(PWM, gyro_lpf_norm)            : %.4f\n", strix_corrf(pwm(mask), gyro_lpf_norm(mask)));
fprintf("corr(PWM, baro_step)                : %.4f\n", strix_corrf(pwm(mask), baro_step(mask)));

raw_std = strix_stdf(raw_acc_norm_g(mask));
acc_p95 = strix_pctf(acc_ned_h(mask), 95);
gyro_p95 = strix_pctf(gyro_lpf_norm(mask), 95);
baro_step_p95 = strix_pctf(baro_step(mask), 95);

sensor_fail = raw_std > raw_acc_std_fail_g || acc_p95 > acc_ned_h_fail || gyro_p95 > gyro_p95_fail;
sensor_warn = raw_std > raw_acc_std_warn_g || acc_p95 > acc_ned_h_warn || gyro_p95 > gyro_p95_warn || baro_step_p95 > baro_step_warn;

fprintf("-------------------------------------------------\n");
fprintf("Sensor status                       : %s\n", strix_status(~sensor_warn, ~sensor_fail));
fprintf("=================================================\n\n");

%% Plots
figure("Name", "01 Sensor Health - Time Series", "Color", "w");
tiledlayout(5, 1, "TileSpacing", "compact");

nexttile;
plot(t, pwm, "LineWidth", 1.0); grid on;
ylabel("PWM [us]");
title("PWM");

nexttile;
plot(t, raw_acc_norm_g, "LineWidth", 1.0); grid on;
ylabel("|acc| [g]");
title("Raw accelerometer norm");

nexttile;
plot(t, acc_ned_h, "LineWidth", 1.0); hold on;
yline(acc_ned_h_warn, "--", "warn");
yline(acc_ned_h_fail, "--r", "fail");
grid on; ylabel("acc H [m/s^2]");
title("Horizontal acceleration after attitude rotation");

nexttile;
plot(t, gyro_raw_norm, "LineWidth", 0.8); hold on;
plot(t, gyro_lpf_norm, "LineWidth", 1.0);
plot(t, gyro_corrected_norm, "LineWidth", 1.0);
yline(gyro_p95_warn, "--", "warn");
yline(gyro_p95_fail, "--r", "fail");
grid on; ylabel("gyro [rad/s]");
legend("raw", "lpf", "corrected", "Location", "best");
title("Gyro norm");

nexttile;
plot(t, baro_rel_alt, "LineWidth", 1.0); hold on;
yyaxis right;
plot(t, baro_age_ms, "LineWidth", 0.8);
grid on; xlabel("time [s]");
ylabel("baro age [ms]");
yyaxis left;
ylabel("baro alt [m]");
title("Barometer relative altitude and age");

figure("Name", "01 Sensor Health - Coupling", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");

nexttile;
scatter(pwm(mask), raw_acc_norm_g(mask), 12, "filled");
grid on; xlabel("PWM [us]"); ylabel("|acc| [g]");
title("PWM vs raw acc norm");

nexttile;
scatter(pwm(mask), acc_ned_h(mask), 12, "filled");
grid on; xlabel("PWM [us]"); ylabel("acc NED H [m/s^2]");
title("PWM vs rotated horizontal acc");

nexttile;
scatter(pwm(mask), gyro_lpf_norm(mask), 12, "filled");
grid on; xlabel("PWM [us]"); ylabel("gyro LPF norm [rad/s]");
title("PWM vs gyro");

nexttile;
scatter(pwm(mask), baro_step(mask), 12, "filled");
grid on; xlabel("PWM [us]"); ylabel("baro step [m/sample]");
title("PWM vs baro step");
