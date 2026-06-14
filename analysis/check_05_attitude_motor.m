clear; clc; close all;

%% STRIX FC Attitude / Motor Check
% Checks attitude stability, gyro rate, and motor balance.
% If setpoint columns exist, also checks tracking error.
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
cfg.title_prefix = "05 Attitude / Motor Check";

roll_pitch_abs_warn = 10.0;      % deg, ground/tethered sanity threshold
gyro_p95_warn = 0.60;            % rad/s
motor_imbalance_warn = 150.0;    % us, max(M)-min(M), tune later

%% Load
log = load_vtol_log(csv_file, cfg);
T = log.T; cols = log.cols; N = log.N; t = log.t; mask = log.mask;

roll = strix_col(T, cols, "ekf_roll_deg", strix_col(T, cols, "roll_deg", nan(N, 1)));
pitch = strix_col(T, cols, "ekf_pitch_deg", strix_col(T, cols, "pitch_deg", nan(N, 1)));
yaw = strix_col(T, cols, "ekf_yaw_deg", strix_col(T, cols, "yaw_deg", nan(N, 1)));

% Try common setpoint names. Missing setpoints are allowed.
roll_sp = first_available(T, cols, N, ["roll_des_deg", "att_roll_sp_deg", "roll_setpoint_deg", "desired_roll_deg"]);
pitch_sp = first_available(T, cols, N, ["pitch_des_deg", "att_pitch_sp_deg", "pitch_setpoint_deg", "desired_pitch_deg"]);
yaw_sp = first_available(T, cols, N, ["yaw_des_deg", "att_yaw_sp_deg", "yaw_setpoint_deg", "desired_yaw_deg"]);

roll_err = roll - roll_sp;
pitch_err = pitch - pitch_sp;
yaw_err = wrapTo180_local(yaw - yaw_sp);

[gyro_raw_norm, gyro_lpf_norm, gyro_corrected_norm, gyro_source] = strix_gyro_norms(T, cols, N);

M1 = strix_col(T, cols, "M1", nan(N, 1));
M2 = strix_col(T, cols, "M2", nan(N, 1));
M3 = strix_col(T, cols, "M3", nan(N, 1));
M4 = strix_col(T, cols, "M4", nan(N, 1));
[pwm_mean, pwm_source] = strix_pwm(T, cols, N);

motors = [M1, M2, M3, M4];
motor_max = max(motors, [], 2, "omitnan");
motor_min = min(motors, [], 2, "omitnan");
motor_imbalance = motor_max - motor_min;
motor_pair_13 = M1 - M3;
motor_pair_24 = M2 - M4;

motor_on = strix_boolcol(T, cols, "motor_on_detected", false(N, 1));
armed = strix_boolcol(T, cols, "ekf_is_armed", false(N, 1));

has_att_sp = any(isfinite(roll_sp)) || any(isfinite(pitch_sp)) || any(isfinite(yaw_sp));

fprintf("PWM source : %s\n", pwm_source);
fprintf("Gyro source: %s\n", gyro_source);
fprintf("Attitude setpoint columns detected: %d\n\n", has_att_sp);

fprintf("=================================================\n");
fprintf("Attitude / Motor Summary\n");
fprintf("=================================================\n");
strix_print_metric("roll_deg", roll(mask), "deg");
strix_print_metric("pitch_deg", pitch(mask), "deg");
strix_print_metric("yaw_deg", yaw(mask), "deg");
strix_print_metric("abs roll", abs(roll(mask)), "deg");
strix_print_metric("abs pitch", abs(pitch(mask)), "deg");
strix_print_metric("gyro_lpf_norm", gyro_lpf_norm(mask), "rad/s");
strix_print_metric("pwm_mean", pwm_mean(mask), "us");
strix_print_metric("motor imbalance max-min", motor_imbalance(mask), "us");
strix_print_metric("M1-M3", motor_pair_13(mask), "us");
strix_print_metric("M2-M4", motor_pair_24(mask), "us");
fprintf("Motor-on samples                   : %d\n", sum(mask & motor_on));
fprintf("Armed samples                      : %d\n", sum(mask & armed));

if has_att_sp
    fprintf("-------------------------------------------------\n");
    strix_print_metric("roll tracking error", roll_err(mask), "deg");
    strix_print_metric("pitch tracking error", pitch_err(mask), "deg");
    strix_print_metric("yaw tracking error", yaw_err(mask), "deg");
else
    fprintf("-------------------------------------------------\n");
    fprintf("No attitude setpoint columns found. Tracking error is skipped.\n");
end

rp_abs_p95 = max(strix_pctf(abs(roll(mask)), 95), strix_pctf(abs(pitch(mask)), 95));
gyro_p95 = strix_pctf(gyro_lpf_norm(mask), 95);
imb_p95 = strix_pctf(motor_imbalance(mask), 95);

att_warn = rp_abs_p95 > roll_pitch_abs_warn || gyro_p95 > gyro_p95_warn || imb_p95 > motor_imbalance_warn;
fprintf("-------------------------------------------------\n");
fprintf("Attitude/motor status              : %s\n", strix_status(~att_warn, true));
fprintf("=================================================\n\n");

%% Plots
figure("Name", "05 Attitude / Motor - Attitude", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, roll, "LineWidth", 1.0); hold on;
if any(isfinite(roll_sp)), plot(t, roll_sp, "--", "LineWidth", 1.0); end
grid on; ylabel("roll [deg]");
title("Roll");
if any(isfinite(roll_sp)), legend("meas", "sp", "Location", "best"); end

nexttile;
plot(t, pitch, "LineWidth", 1.0); hold on;
if any(isfinite(pitch_sp)), plot(t, pitch_sp, "--", "LineWidth", 1.0); end
grid on; ylabel("pitch [deg]");
title("Pitch");
if any(isfinite(pitch_sp)), legend("meas", "sp", "Location", "best"); end

nexttile;
plot(t, yaw, "LineWidth", 1.0); hold on;
if any(isfinite(yaw_sp)), plot(t, yaw_sp, "--", "LineWidth", 1.0); end
grid on; ylabel("yaw [deg]");
title("Yaw");
if any(isfinite(yaw_sp)), legend("meas", "sp", "Location", "best"); end

nexttile;
plot(t, gyro_lpf_norm, "LineWidth", 1.0);
grid on; xlabel("time [s]"); ylabel("gyro [rad/s]");
title("Gyro LPF norm");

figure("Name", "05 Attitude / Motor - Motors", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, M1, "LineWidth", 1.0); hold on;
plot(t, M2, "LineWidth", 1.0);
plot(t, M3, "LineWidth", 1.0);
plot(t, M4, "LineWidth", 1.0);
grid on; ylabel("PWM [us]");
legend("M1", "M2", "M3", "M4", "Location", "best");
title("Motor outputs");

nexttile;
plot(t, pwm_mean, "LineWidth", 1.0); hold on;
stairs(t, double(motor_on) * strix_maxf(pwm_mean), "LineWidth", 0.8);
grid on; ylabel("PWM [us]");
title("PWM mean and motor-on flag");

nexttile;
plot(t, motor_imbalance, "LineWidth", 1.0); hold on;
yline(motor_imbalance_warn, "--r", "warn");
grid on; ylabel("max-min [us]");
title("Motor imbalance");

nexttile;
plot(t, motor_pair_13, "LineWidth", 1.0); hold on;
plot(t, motor_pair_24, "LineWidth", 1.0);
grid on; xlabel("time [s]"); ylabel("pair diff [us]");
legend("M1-M3", "M2-M4", "Location", "best");
title("Motor pair differences");

figure("Name", "05 Attitude / Motor - Coupling", "Color", "w");
tiledlayout(1, 3, "TileSpacing", "compact");

nexttile;
scatter(pwm_mean(mask), abs(roll(mask)), 12, "filled");
grid on; xlabel("PWM mean [us]"); ylabel("|roll| [deg]");
title("PWM vs roll");

nexttile;
scatter(pwm_mean(mask), abs(pitch(mask)), 12, "filled");
grid on; xlabel("PWM mean [us]"); ylabel("|pitch| [deg]");
title("PWM vs pitch");

nexttile;
scatter(pwm_mean(mask), motor_imbalance(mask), 12, "filled");
grid on; xlabel("PWM mean [us]"); ylabel("motor imbalance [us]");
title("PWM vs motor imbalance");

%% Local functions
function y = first_available(T, cols, N, names)
    y = nan(N, 1);
    for i = 1:numel(names)
        if ismember(names(i), cols)
            y = double(T.(names(i)));
            return;
        end
    end
end

function a = wrapTo180_local(a)
    a = mod(a + 180, 360) - 180;
end
