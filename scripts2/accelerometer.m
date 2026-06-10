clear; clc; close all;

%% ============================================================
%  STRIX FC Presentation-Focused Accel Contamination Analysis
% =============================================================
% 목적:
%   발표자료에 넣을 수 있을 정도로 핵심 plot만 생성한다.
%
% 전달하고 싶은 메시지:
%   "댐핑 전에는 PWM 상승/진동 구간에서 가속도계 오염이 커졌고,
%    이 오염이 acc_ned_h 및 EKF velocity spike로 이어졌다.
%    댐핑 후에는 raw accel / propagation accel / velocity spike가 줄었다."
%
% 사용법:
%   csv_file만 바꿔서 damping 전/후 각각 실행.
%
% 출력:
%   results/<csv>_presentation_report.csv
%   results/<csv>_presentation_report.mat
%   results/<csv>_P01_accel_pipeline.png
%   results/<csv>_P02_accel_distribution.png
%   results/<csv>_P03_velocity_effect.png
%   results/<csv>_P04_event_zoom.png

%% ============================================================
% 0. User Settings
% =============================================================

csv_file = "260610\fc_damp_04.CSV";
% csv_file = "data\vel_ctrl_3.CSV";

out_dir = "results";

eval_start_sec = 0.0;
eval_end_sec   = inf;

min_fix_type = 3;
min_num_sats = 6;
use_ekf_ready = true;
use_gnss_ref_ready = true;

% 발표용 threshold
pwm_high_threshold = 1400.0;
raw_acc_norm_th_g  = 2.0;
acc_h_threshold    = 3.0;
speed_spike_th     = 0.5;
gnss_speed_low_th  = 0.3;

% zoom
zoom_before_sec = 4.0;
zoom_after_sec  = 4.0;

save_figures = true;
save_report  = true;

% ===================== Figure 창 제어 =====================
% false: figure 창 안 띄우고 PNG 저장만 함
% true : figure 창도 띄움
show_figures = true;

% show_figures = true일 때만 의미 있음
% true : MATLAB 내부 탭으로 figure 정리
% false: Windows 독립 창으로 figure 표시
dock_figures = true;

if show_figures
    set(0, "DefaultFigureVisible", "on");

    if dock_figures
        set(0, "DefaultFigureWindowStyle", "docked");
    else
        set(0, "DefaultFigureWindowStyle", "normal");
    end
else
    set(0, "DefaultFigureVisible", "off");
end

g0 = 9.80665;

%% ============================================================
% 1. Load CSV
% =============================================================

if ~isfile(csv_file)
    error("CSV 파일을 찾을 수 없습니다: %s", csv_file);
end

if ~isfolder(out_dir)
    mkdir(out_dir);
end

T = readtable(csv_file, "VariableNamingRule", "preserve");
cols = string(T.Properties.VariableNames);
N = height(T);

[~, csv_base, ~] = fileparts(csv_file);
csv_base = matlab.lang.makeValidName(csv_base);

fprintf("\n=================================================\n");
fprintf("STRIX FC Presentation-Focused Accel Analysis\n");
fprintf("=================================================\n");
fprintf("Loaded CSV : %s\n", csv_file);
fprintf("Rows       : %d\n", N);
fprintf("Columns    : %d\n", width(T));

need_cols(cols, "timestamp_ms");

t = double(T.timestamp_ms) * 1e-3;
t = t - t(1);

eval_mask = t >= eval_start_sec & t <= eval_end_sec;

%% ============================================================
% 2. Load Signals
% =============================================================

need_cols(cols, ["ax","ay","az"]);
need_cols(cols, ["ekf_accel_body_x","ekf_accel_body_y","ekf_accel_body_z"]);
need_cols(cols, ["acc_ned_n","acc_ned_e","acc_ned_d"]);
need_cols(cols, ["ekf_vel_n","ekf_vel_e","ekf_vel_d"]);
need_cols(cols, ["gnss_vel_n_mps","gnss_vel_e_mps","gnss_vel_d_mps"]);
need_cols(cols, ["ekf_pos_n","ekf_pos_e","ekf_pos_d"]);
need_cols(cols, ["ekf_gnss_pos_n","ekf_gnss_pos_e","ekf_gnss_pos_d"]);

% Raw accel
raw_acc_g = [double(T.ax), double(T.ay), double(T.az)];
raw_acc_norm_g = vecnorm(raw_acc_g, 2, 2);

% Filtered/body accel
acc_body = [ ...
    double(T.ekf_accel_body_x), ...
    double(T.ekf_accel_body_y), ...
    double(T.ekf_accel_body_z) ...
];
acc_body_norm = vecnorm(acc_body, 2, 2);

% Propagation accel
acc_ned = [double(T.acc_ned_n), double(T.acc_ned_e), double(T.acc_ned_d)];
acc_ned_h = hypot(acc_ned(:,1), acc_ned(:,2));

% Velocity
ekf_vel = [double(T.ekf_vel_n), double(T.ekf_vel_e), double(T.ekf_vel_d)];
gnss_vel = [double(T.gnss_vel_n_mps), double(T.gnss_vel_e_mps), double(T.gnss_vel_d_mps)];

ekf_speed_h = hypot(ekf_vel(:,1), ekf_vel(:,2));
gnss_speed_h = hypot(gnss_vel(:,1), gnss_vel(:,2));
vel_diff_h = hypot(gnss_vel(:,1) - ekf_vel(:,1), gnss_vel(:,2) - ekf_vel(:,2));

% Position
ekf_pos = [double(T.ekf_pos_n), double(T.ekf_pos_e), double(T.ekf_pos_d)];
gnss_pos = [double(T.ekf_gnss_pos_n), double(T.ekf_gnss_pos_e), double(T.ekf_gnss_pos_d)];

pos_err = gnss_pos - ekf_pos;
pos_err_h = hypot(pos_err(:,1), pos_err(:,2));

% Optional
gnss_hacc = col_or(T, cols, "gnss_hacc_m", nan(N,1));
gnss_sacc = col_or(T, cols, "gnss_sacc_mps", nan(N,1));

if ismember("ekf_pwm_mean", cols)
    pwm = double(T.ekf_pwm_mean);
elseif all(ismember(["M1","M2","M3","M4"], cols))
    pwm = mean([double(T.M1), double(T.M2), double(T.M3), double(T.M4)], 2, "omitnan");
else
    pwm = nan(N,1);
    warning("PWM 컬럼을 찾지 못했습니다.");
end

roll_deg  = col_or(T, cols, "roll_deg",  col_or(T, cols, "ekf_roll_deg",  nan(N,1)));
pitch_deg = col_or(T, cols, "pitch_deg", col_or(T, cols, "ekf_pitch_deg", nan(N,1)));
yaw_deg   = col_or(T, cols, "yaw_deg",   col_or(T, cols, "ekf_yaw_deg",   nan(N,1)));

gnss_update_executed = bool_col(T, cols, "gnss_update_executed", false(N,1));
gnss_accepted        = bool_col(T, cols, "gnss_correction_accepted", false(N,1));
gnss_velocity_used   = bool_col(T, cols, "gnss_velocity_used", false(N,1));
zupt_active          = bool_col(T, cols, "zupt_active", false(N,1));

delta_vel_gnss = [ ...
    col_or(T, cols, "delta_vel_gnss_update_n", zeros(N,1)), ...
    col_or(T, cols, "delta_vel_gnss_update_e", zeros(N,1)), ...
    col_or(T, cols, "delta_vel_gnss_update_d", zeros(N,1)) ...
];
delta_vel_gnss_h = hypot(delta_vel_gnss(:,1), delta_vel_gnss(:,2));

delta_vel_zupt = [ ...
    col_or(T, cols, "delta_vel_zupt_update_n", zeros(N,1)), ...
    col_or(T, cols, "delta_vel_zupt_update_e", zeros(N,1)), ...
    col_or(T, cols, "delta_vel_zupt_update_d", zeros(N,1)) ...
];
delta_vel_zupt_h = hypot(delta_vel_zupt(:,1), delta_vel_zupt(:,2));

%% ============================================================
% 3. Valid Mask
% =============================================================

gnss_valid = all(isfinite(gnss_pos), 2);

if ismember("gnss_valid", cols)
    gnss_valid = gnss_valid & double(T.gnss_valid) ~= 0;
end

if ismember("gnss_fix_type", cols)
    gnss_valid = gnss_valid & double(T.gnss_fix_type) >= min_fix_type;
end

if ismember("numSV", cols)
    gnss_valid = gnss_valid & double(T.numSV) >= min_num_sats;
elseif ismember("gnss_num_sats", cols)
    gnss_valid = gnss_valid & double(T.gnss_num_sats) >= min_num_sats;
end

if use_gnss_ref_ready && ismember("gnss_ref_ready", cols)
    gnss_valid = gnss_valid & double(T.gnss_ref_ready) ~= 0;
end

gnss_valid = gnss_valid & vecnorm(gnss_pos, 2, 2) > 1e-9;

ekf_valid = all(isfinite(ekf_pos), 2) & all(isfinite(ekf_vel), 2);

if use_ekf_ready && ismember("ekf_ready", cols)
    ekf_valid = ekf_valid & double(T.ekf_ready) ~= 0;
end

mask = eval_mask & gnss_valid & ekf_valid;

if sum(mask) < 5
    error("평가 가능한 샘플이 너무 적습니다. mask count = %d", sum(mask));
end

%% ============================================================
% 4. Classification
% =============================================================

acc_disturbance = ...
    (raw_acc_norm_g > raw_acc_norm_th_g) | ...
    (acc_ned_h > acc_h_threshold);

high_pwm_acc_disturbance = ...
    (pwm > pwm_high_threshold) & ...
    (acc_ned_h > acc_h_threshold) & ...
    (gnss_speed_h < gnss_speed_low_th);

velocity_spike = ekf_speed_h >= speed_spike_th;

normal_mask = mask & ~acc_disturbance;
acc_dist_mask = mask & acc_disturbance;
high_pwm_dist_mask = mask & high_pwm_acc_disturbance;
spike_mask = mask & velocity_spike;

% 발표용 대표 이벤트: high PWM disturbance 중 EKF speed 최대
event_candidate = high_pwm_dist_mask;

if ~any(event_candidate)
    event_candidate = acc_dist_mask;
end

if ~any(event_candidate)
    event_candidate = mask;
end

tmp = ekf_speed_h;
tmp(~event_candidate) = -inf;
[~, event_idx] = max(tmp);

event_time = t(event_idx);

zoom_mask = t >= event_time - zoom_before_sec & ...
            t <= event_time + zoom_after_sec;

%% ============================================================
% 5. Summary
% =============================================================

S = struct();

S.csv_file = string(csv_file);
S.rows = N;
S.eval_samples = sum(mask);
S.eval_t_start = min(t(mask));
S.eval_t_end = max(t(mask));

S.normal_samples = sum(normal_mask);
S.acc_dist_samples = sum(acc_dist_mask);
S.high_pwm_dist_samples = sum(high_pwm_dist_mask);
S.velocity_spike_samples = sum(spike_mask);

S.acc_dist_ratio = sum(acc_dist_mask) / sum(mask);
S.high_pwm_dist_ratio = sum(high_pwm_dist_mask) / sum(mask);
S.velocity_spike_ratio = sum(spike_mask) / sum(mask);

S.raw_acc_norm_g = stat_pos(raw_acc_norm_g(mask));
S.acc_body_norm = stat_pos(acc_body_norm(mask));
S.acc_ned_h = stat_pos(acc_ned_h(mask));
S.ekf_speed_h = stat_pos(ekf_speed_h(mask));
S.gnss_speed_h = stat_pos(gnss_speed_h(mask));
S.vel_diff_h = stat_pos(vel_diff_h(mask));
S.pos_err_h = stat_pos(pos_err_h(mask));
S.gnss_hacc = stat_pos(gnss_hacc(mask));
S.gnss_sacc = stat_pos(gnss_sacc(mask));

S.raw_acc_over_th_count = sum(mask & raw_acc_norm_g > raw_acc_norm_th_g);
S.acc_ned_h_over_th_count = sum(mask & acc_ned_h > acc_h_threshold);
S.ekf_speed_over_th_count = sum(mask & ekf_speed_h > speed_spike_th);

S.raw_acc_over_th_ratio = S.raw_acc_over_th_count / S.eval_samples;
S.acc_ned_h_over_th_ratio = S.acc_ned_h_over_th_count / S.eval_samples;
S.ekf_speed_over_th_ratio = S.ekf_speed_over_th_count / S.eval_samples;

S.gnss_update_executed_count = sum(gnss_update_executed & mask);
S.gnss_accepted_count = sum(gnss_accepted & mask);
S.gnss_velocity_used_count = sum(gnss_velocity_used & mask);
S.zupt_active_count = sum(zupt_active & mask);

S.event_time = event_time;
S.event_pwm = pwm(event_idx);
S.event_raw_acc_norm_g = raw_acc_norm_g(event_idx);
S.event_acc_body_norm = acc_body_norm(event_idx);
S.event_acc_ned_h = acc_ned_h(event_idx);
S.event_ekf_speed_h = ekf_speed_h(event_idx);
S.event_gnss_speed_h = gnss_speed_h(event_idx);
S.event_vel_diff_h = vel_diff_h(event_idx);
S.event_pos_err_h = pos_err_h(event_idx);
S.event_roll_deg = roll_deg(event_idx);
S.event_pitch_deg = pitch_deg(event_idx);
S.event_yaw_deg = yaw_deg(event_idx);
S.event_delta_vel_gnss_h = delta_vel_gnss_h(event_idx);
S.event_delta_vel_zupt_h = delta_vel_zupt_h(event_idx);

fprintf("\n=================================================\n");
fprintf("Presentation Summary\n");
fprintf("=================================================\n");
fprintf("Evaluation samples              : %d\n", S.eval_samples);
fprintf("Evaluation time range           : %.3f ~ %.3f sec\n", S.eval_t_start, S.eval_t_end);
fprintf("acc disturbance samples         : %d / %d (%.2f %%)\n", ...
    S.acc_dist_samples, S.eval_samples, 100*S.acc_dist_ratio);
fprintf("high PWM acc disturbance samples: %d / %d (%.2f %%)\n", ...
    S.high_pwm_dist_samples, S.eval_samples, 100*S.high_pwm_dist_ratio);
fprintf("velocity spike samples          : %d / %d (%.2f %%)\n", ...
    S.velocity_spike_samples, S.eval_samples, 100*S.velocity_spike_ratio);

print_line_stats("raw_acc_norm_g", S.raw_acc_norm_g, "g");
print_line_stats("acc_body_norm", S.acc_body_norm, "m/s^2");
print_line_stats("acc_ned_h", S.acc_ned_h, "m/s^2");
print_line_stats("ekf_speed_h", S.ekf_speed_h, "m/s");
print_line_stats("vel_diff_h", S.vel_diff_h, "m/s");
print_line_stats("pos_err_h", S.pos_err_h, "m");

fprintf("\nRepresentative event\n");
fprintf("time              : %.3f sec\n", S.event_time);
fprintf("PWM               : %.2f\n", S.event_pwm);
fprintf("raw accel norm    : %.4f g\n", S.event_raw_acc_norm_g);
fprintf("acc body norm     : %.4f m/s^2\n", S.event_acc_body_norm);
fprintf("acc ned H         : %.4f m/s^2\n", S.event_acc_ned_h);
fprintf("EKF speed H       : %.4f m/s\n", S.event_ekf_speed_h);
fprintf("GNSS speed H      : %.4f m/s\n", S.event_gnss_speed_h);
fprintf("vel diff H        : %.4f m/s\n", S.event_vel_diff_h);
fprintf("pos err H         : %.4f m\n", S.event_pos_err_h);
fprintf("roll/pitch/yaw    : %.3f / %.3f / %.3f deg\n", ...
    S.event_roll_deg, S.event_pitch_deg, S.event_yaw_deg);
fprintf("dV GNSS / ZUPT H  : %.4f / %.4f m/s\n", ...
    S.event_delta_vel_gnss_h, S.event_delta_vel_zupt_h);

%% ============================================================
% 6. Report Table
% =============================================================

metric = strings(0,1);
value = [];
unit = strings(0,1);

[metric, value, unit] = add_metric(metric, value, unit, "eval_samples", S.eval_samples, "samples");
[metric, value, unit] = add_metric(metric, value, unit, "eval_t_start", S.eval_t_start, "s");
[metric, value, unit] = add_metric(metric, value, unit, "eval_t_end", S.eval_t_end, "s");

[metric, value, unit] = add_stats_to_table(metric, value, unit, "raw_acc_norm_g", S.raw_acc_norm_g, "g");
[metric, value, unit] = add_stats_to_table(metric, value, unit, "acc_body_norm", S.acc_body_norm, "m/s^2");
[metric, value, unit] = add_stats_to_table(metric, value, unit, "acc_ned_h", S.acc_ned_h, "m/s^2");
[metric, value, unit] = add_stats_to_table(metric, value, unit, "ekf_speed_h", S.ekf_speed_h, "m/s");
[metric, value, unit] = add_stats_to_table(metric, value, unit, "vel_diff_h", S.vel_diff_h, "m/s");
[metric, value, unit] = add_stats_to_table(metric, value, unit, "pos_err_h", S.pos_err_h, "m");
[metric, value, unit] = add_stats_to_table(metric, value, unit, "gnss_hacc", S.gnss_hacc, "m");
[metric, value, unit] = add_stats_to_table(metric, value, unit, "gnss_sacc", S.gnss_sacc, "m/s");

[metric, value, unit] = add_metric(metric, value, unit, "acc_dist_samples", S.acc_dist_samples, "samples");
[metric, value, unit] = add_metric(metric, value, unit, "acc_dist_ratio", 100*S.acc_dist_ratio, "%");
[metric, value, unit] = add_metric(metric, value, unit, "high_pwm_dist_samples", S.high_pwm_dist_samples, "samples");
[metric, value, unit] = add_metric(metric, value, unit, "high_pwm_dist_ratio", 100*S.high_pwm_dist_ratio, "%");
[metric, value, unit] = add_metric(metric, value, unit, "velocity_spike_samples", S.velocity_spike_samples, "samples");
[metric, value, unit] = add_metric(metric, value, unit, "velocity_spike_ratio", 100*S.velocity_spike_ratio, "%");

[metric, value, unit] = add_metric(metric, value, unit, "raw_acc_over_threshold_ratio", 100*S.raw_acc_over_th_ratio, "%");
[metric, value, unit] = add_metric(metric, value, unit, "acc_ned_h_over_threshold_ratio", 100*S.acc_ned_h_over_th_ratio, "%");
[metric, value, unit] = add_metric(metric, value, unit, "ekf_speed_over_threshold_ratio", 100*S.ekf_speed_over_th_ratio, "%");

[metric, value, unit] = add_metric(metric, value, unit, "event_time", S.event_time, "s");
[metric, value, unit] = add_metric(metric, value, unit, "event_pwm", S.event_pwm, "pwm");
[metric, value, unit] = add_metric(metric, value, unit, "event_raw_acc_norm_g", S.event_raw_acc_norm_g, "g");
[metric, value, unit] = add_metric(metric, value, unit, "event_acc_ned_h", S.event_acc_ned_h, "m/s^2");
[metric, value, unit] = add_metric(metric, value, unit, "event_ekf_speed_h", S.event_ekf_speed_h, "m/s");
[metric, value, unit] = add_metric(metric, value, unit, "event_vel_diff_h", S.event_vel_diff_h, "m/s");
[metric, value, unit] = add_metric(metric, value, unit, "event_pos_err_h", S.event_pos_err_h, "m");

report_table = table(metric(:), value(:), unit(:), ...
    'VariableNames', {'metric', 'value', 'unit'});

if save_report
    csv_report_path = fullfile(out_dir, csv_base + "_presentation_report.csv");
    mat_report_path = fullfile(out_dir, csv_base + "_presentation_report.mat");

    writetable(report_table, csv_report_path);

    save(mat_report_path, ...
        "S", "report_table", ...
        "t", "mask", "normal_mask", "acc_dist_mask", "high_pwm_dist_mask", "spike_mask", ...
        "pwm", ...
        "raw_acc_norm_g", "acc_body_norm", "acc_ned_h", ...
        "ekf_speed_h", "gnss_speed_h", "vel_diff_h", ...
        "pos_err_h", "gnss_hacc", "gnss_sacc", ...
        "event_time", "event_idx");

    fprintf("\nSaved report CSV: %s\n", csv_report_path);
    fprintf("Saved report MAT: %s\n", mat_report_path);
end

%% ============================================================
% 7. Presentation Figures
% =============================================================

%% ------------------------------------------------------------
% P01. Accel Pipeline
% ------------------------------------------------------------

fig1 = figure("Name", "P01 Accel Pipeline");
set(fig1, "Color", "w", "Position", [100 100 1100 750]);

tiledlayout(3,1, "TileSpacing", "compact", "Padding", "compact");

nexttile; hold on; grid on;
plot(t(mask), pwm(mask), "LineWidth", 1.2);
yline(pwm_high_threshold, "--", "High PWM threshold");
xline(event_time, ":", "Event");
ylabel("PWM");
title("Motor output");

nexttile; hold on; grid on;
plot(t(mask), raw_acc_norm_g(mask), "LineWidth", 1.2);
yline(1.0, "--", "1 g");
yline(raw_acc_norm_th_g, ":", "Raw accel threshold");
xline(event_time, ":", "Event");
ylabel("|raw accel| [g]");
title("Raw accelerometer norm");

nexttile; hold on; grid on;
plot(t(mask), acc_ned_h(mask), "LineWidth", 1.2);
yline(acc_h_threshold, ":", "Propagation accel threshold");
xline(event_time, ":", "Event");
ylabel("acc\_ned H [m/s^2]");
xlabel("Time [s]");
title("Horizontal acceleration used for EKF propagation");

save_fig_if_needed(fig1, out_dir, csv_base + "_P01_accel_pipeline.png", save_figures);

%% ------------------------------------------------------------
% P02. Accel Distribution
% ------------------------------------------------------------

fig2 = figure("Name", "P02 Accel Distribution");
set(fig2, "Color", "w", "Position", [150 150 1100 650]);

tiledlayout(1,2, "TileSpacing", "compact", "Padding", "compact");

nexttile; hold on; grid on;
histogram(raw_acc_norm_g(mask), 45);
xline(raw_acc_norm_th_g, ":", "Threshold", "LineWidth", 1.2);
xline(S.raw_acc_norm_g.p95, "--", sprintf("p95 = %.2f g", S.raw_acc_norm_g.p95), "LineWidth", 1.2);
xlabel("|raw accel| [g]");
ylabel("Count");
title("Raw accel contamination distribution");

nexttile; hold on; grid on;
histogram(acc_ned_h(mask), 45);
xline(acc_h_threshold, ":", "Threshold", "LineWidth", 1.2);
xline(S.acc_ned_h.p95, "--", sprintf("p95 = %.2f m/s^2", S.acc_ned_h.p95), "LineWidth", 1.2);
xlabel("acc\_ned H [m/s^2]");
ylabel("Count");
title("Propagation acceleration distribution");

save_fig_if_needed(fig2, out_dir, csv_base + "_P02_accel_distribution.png", save_figures);

%% ------------------------------------------------------------
% P03. Velocity Effect
% ------------------------------------------------------------

fig3 = figure("Name", "P03 Velocity Effect");
set(fig3, "Color", "w", "Position", [200 200 1100 750]);

tiledlayout(3,1, "TileSpacing", "compact", "Padding", "compact");

nexttile; hold on; grid on;
plot(t(mask), acc_ned_h(mask), "LineWidth", 1.2);
yline(acc_h_threshold, ":", "Accel threshold");
xline(event_time, ":", "Event");
ylabel("acc\_ned H [m/s^2]");
title("EKF propagation acceleration");

nexttile; hold on; grid on;
plot(t(mask), ekf_speed_h(mask), "LineWidth", 1.2);
plot(t(mask), gnss_speed_h(mask), "--", "LineWidth", 1.2);
yline(speed_spike_th, ":", "Velocity spike threshold");
xline(event_time, ":", "Event");
ylabel("Speed H [m/s]");
title("EKF speed vs GNSS speed");
legend("EKF", "GNSS", "Location", "best");

nexttile; hold on; grid on;
plot(t(mask), vel_diff_h(mask), "LineWidth", 1.2);
xline(event_time, ":", "Event");
ylabel("|GNSS - EKF| [m/s]");
xlabel("Time [s]");
title("Horizontal velocity disagreement");

save_fig_if_needed(fig3, out_dir, csv_base + "_P03_velocity_effect.png", save_figures);

%% ------------------------------------------------------------
% P04. Representative Event Zoom
% ------------------------------------------------------------

fig4 = figure("Name", "P04 Representative Event Zoom");
set(fig4, "Color", "w", "Position", [250 250 1100 850]);

tiledlayout(4,1, "TileSpacing", "compact", "Padding", "compact");

nexttile; hold on; grid on;
plot(t(zoom_mask), pwm(zoom_mask), "LineWidth", 1.2);
yline(pwm_high_threshold, "--", "High PWM");
xline(event_time, ":", "Event");
ylabel("PWM");
title(sprintf("Representative event zoom: %.3f sec", event_time));

nexttile; hold on; grid on;
plot(t(zoom_mask), raw_acc_norm_g(zoom_mask), "LineWidth", 1.2);
yline(1.0, "--", "1 g");
yline(raw_acc_norm_th_g, ":", "Threshold");
xline(event_time, ":", "Event");
ylabel("|raw| [g]");
title("Raw accelerometer norm");

nexttile; hold on; grid on;
plot(t(zoom_mask), acc_ned_h(zoom_mask), "LineWidth", 1.2);
yline(acc_h_threshold, ":", "Threshold");
xline(event_time, ":", "Event");
ylabel("acc H [m/s^2]");
title("Horizontal propagation acceleration");

nexttile; hold on; grid on;
plot(t(zoom_mask), ekf_speed_h(zoom_mask), "LineWidth", 1.2);
plot(t(zoom_mask), gnss_speed_h(zoom_mask), "--", "LineWidth", 1.2);
yline(speed_spike_th, ":", "Spike threshold");
xline(event_time, ":", "Event");
ylabel("Speed H [m/s]");
xlabel("Time [s]");
title("EKF velocity spike compared with GNSS speed");
legend("EKF", "GNSS", "Location", "best");

save_fig_if_needed(fig4, out_dir, csv_base + "_P04_event_zoom.png", save_figures);

% show_figures = false일 때 메모리/창 정리
if ~show_figures
    close(fig1);
    close(fig2);
    close(fig3);
    close(fig4);
end

fprintf("\nDone.\n");
fprintf("발표자료용 figure:\n");
fprintf("  P01_accel_pipeline       : PWM -> raw accel -> acc_ned_h\n");
fprintf("  P02_accel_distribution   : raw accel / propagation accel 분포\n");
fprintf("  P03_velocity_effect      : acc_ned_h -> EKF speed spike\n");
fprintf("  P04_event_zoom           : 대표 이벤트 확대\n\n");

%% ============================================================
% Local Functions
% =============================================================

function need_cols(cols, names)
    names = string(names);
    missing = names(~ismember(names, cols));
    if ~isempty(missing)
        error("필수 컬럼이 없습니다: %s", strjoin(missing, ", "));
    end
end

function x = col_or(T, cols, name, default_value)
    name = string(name);
    if ismember(name, cols)
        x = double(T.(name));
    else
        x = default_value;
    end
end

function b = bool_col(T, cols, name, default_value)
    name = string(name);
    if ismember(name, cols)
        b = double(T.(name)) ~= 0;
    else
        b = default_value;
    end
end

function s = stat_pos(x)
    x = x(:);
    x = x(isfinite(x));

    if isempty(x)
        s.mean = NaN;
        s.median = NaN;
        s.std = NaN;
        s.rms = NaN;
        s.p50 = NaN;
        s.p95 = NaN;
        s.p99 = NaN;
        s.min = NaN;
        s.max = NaN;
        return;
    end

    s.mean = mean(x);
    s.median = median(x);
    s.std = std(x);
    s.rms = sqrt(mean(x.^2));
    s.p50 = prctile(x, 50);
    s.p95 = prctile(x, 95);
    s.p99 = prctile(x, 99);
    s.min = min(x);
    s.max = max(x);
end

function print_line_stats(name, s, unit)
    fprintf("%-18s mean/rms/p95/max = %9.4f / %9.4f / %9.4f / %9.4f [%s]\n", ...
        name, s.mean, s.rms, s.p95, s.max, unit);
end

function [metric, value, unit] = add_metric(metric, value, unit, name, val, unit_str)
    metric(end+1,1) = string(name);
    value(end+1,1) = val;
    unit(end+1,1) = string(unit_str);
end

function [metric, value, unit] = add_stats_to_table(metric, value, unit, prefix, s, unit_str)
    fields = ["mean","median","std","rms","p95","p99","min","max"];
    for k = 1:numel(fields)
        f = fields(k);
        [metric, value, unit] = add_metric(metric, value, unit, ...
            prefix + "_" + f, s.(f), unit_str);
    end
end

function save_fig_if_needed(fig_handle, out_dir, filename, save_figures)
    if save_figures
        exportgraphics(fig_handle, fullfile(out_dir, filename), "Resolution", 220);
    end
end