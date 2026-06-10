clear; clc; close all;

%% ============================================================
%  STRIX FC Final Single-CSV Damping Analysis
% =============================================================
% 목적:
%   이 스크립트는 CSV 1개를 독립적으로 분석한다.
%   vel_ctrl_3.CSV, fc_damp_04.CSV 등을 각각 따로 실행한 뒤
%   출력된 report와 figure를 비교해서 damping 전/후를 설명하는 용도다.
%
% 핵심 질문:
%   1) raw accel이 PWM/진동 구간에서 오염되는가?
%   2) LPF/bias 이후 ekf_accel_body에서 줄었는가?
%   3) 실제 propagation 입력인 acc_ned_h가 얼마나 커지는가?
%   4) acc_ned_h 증가가 EKF velocity spike로 이어지는가?
%   5) GNSS velocity / ZUPT correction이 spike를 얼마나 잡는가?
%   6) 결과적으로 position/velocity 안정성이 임무에 충분한가?
%
% 사용법:
%   아래 csv_file만 바꿔서 각각 실행.
%
%   csv_file = "data\vel_ctrl_3.CSV";     % damping 전
%   csv_file = "260610\fc_damp_04.CSV";   % damping 후
%
% 출력:
%   - 콘솔 요약
%   - results/<csv이름>_final_analysis_report.csv
%   - results/<csv이름>_final_analysis_report.mat
%   - results/<csv이름>_figXX_*.png
%
% 작성 의도:
%   두 CSV를 한 코드에서 직접 비교하지 않는다.
%   같은 분석 기준으로 각 CSV를 따로 돌리고, 결과 수치를 전/후 설명에 사용한다.

%% ============================================================
% 0. User Settings
% =============================================================

% csv_file = "260610\fc_damp_04.CSV";
csv_file = "data\vel_ctrl_3.CSV";

out_dir = "results";

eval_start_sec = 0.0;
eval_end_sec   = inf;

% GNSS/EKF valid 조건
min_fix_type = 3;
min_num_sats = 6;
use_ekf_ready = true;
use_gnss_ref_ready = true;

% 속도 제어 구간만 별도로 보고 싶으면 true.
% vel_ctrl_valid 컬럼이 있고 1인 샘플이 충분할 때만 적용된다.
use_vel_ctrl_valid_for_eval = false;

% disturbance threshold
pwm_high_threshold = 1400.0;   % 고출력 판단 PWM
acc_h_threshold    = 3.0;      % 실제 propagation 수평가속도 외란 기준 [m/s^2]
raw_acc_norm_th_g  = 2.0;      % raw accel norm 외란 기준 [g]
gnss_speed_low_th  = 0.3;      % GNSS 정지 판단 [m/s]
speed_spike_th     = 0.5;      % EKF horizontal velocity spike 기준 [m/s]

% zoom figure 범위
zoom_before_sec = 5.0;
zoom_after_sec  = 5.0;

% Figure 저장 여부
save_figures = true;
save_report  = true;

% plot 표시 여부. batch 실행이면 false로 두면 됨.
show_figures = true;

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
fprintf("STRIX FC Final Single-CSV Damping Analysis\n");
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

% Required columns
need_cols(cols, ["ax","ay","az"]);
need_cols(cols, ["ekf_accel_body_x","ekf_accel_body_y","ekf_accel_body_z"]);
need_cols(cols, ["acc_ned_n","acc_ned_e","acc_ned_d"]);
need_cols(cols, ["ekf_vel_n","ekf_vel_e","ekf_vel_d"]);
need_cols(cols, ["gnss_vel_n_mps","gnss_vel_e_mps","gnss_vel_d_mps"]);
need_cols(cols, ["ekf_pos_n","ekf_pos_e","ekf_pos_d"]);
need_cols(cols, ["ekf_gnss_pos_n","ekf_gnss_pos_e","ekf_gnss_pos_d"]);

% Accel pipeline
raw_acc_g = [double(T.ax), double(T.ay), double(T.az)];
raw_acc_mps2 = raw_acc_g * g0;
raw_acc_norm_g = vecnorm(raw_acc_g, 2, 2);
raw_acc_norm_mps2 = raw_acc_norm_g * g0;

acc_body = [ ...
    double(T.ekf_accel_body_x), ...
    double(T.ekf_accel_body_y), ...
    double(T.ekf_accel_body_z) ...
];
acc_body_norm = vecnorm(acc_body, 2, 2);

acc_ned = [double(T.acc_ned_n), double(T.acc_ned_e), double(T.acc_ned_d)];
acc_ned_h = hypot(acc_ned(:,1), acc_ned(:,2));
acc_ned_norm = vecnorm(acc_ned, 2, 2);

% Velocity
ekf_vel = [double(T.ekf_vel_n), double(T.ekf_vel_e), double(T.ekf_vel_d)];
gnss_vel = [double(T.gnss_vel_n_mps), double(T.gnss_vel_e_mps), double(T.gnss_vel_d_mps)];

ekf_speed_h = hypot(ekf_vel(:,1), ekf_vel(:,2));
gnss_speed_h = hypot(gnss_vel(:,1), gnss_vel(:,2));
vel_diff = gnss_vel - ekf_vel;
vel_diff_h = hypot(vel_diff(:,1), vel_diff(:,2));

% Position
ekf_pos = [double(T.ekf_pos_n), double(T.ekf_pos_e), double(T.ekf_pos_d)];
gnss_pos = [double(T.ekf_gnss_pos_n), double(T.ekf_gnss_pos_e), double(T.ekf_gnss_pos_d)];
pos_err = gnss_pos - ekf_pos;
pos_err_h = hypot(pos_err(:,1), pos_err(:,2));

% Optional accuracy / correction / state flags
gnss_hacc = col_or(T, cols, "gnss_hacc_m", nan(N,1));
gnss_vacc = col_or(T, cols, "gnss_vacc_m", nan(N,1));
gnss_sacc = col_or(T, cols, "gnss_sacc_mps", nan(N,1));

if ismember("ekf_pwm_mean", cols)
    pwm = double(T.ekf_pwm_mean);
elseif all(ismember(["M1","M2","M3","M4"], cols))
    pwm = mean([double(T.M1), double(T.M2), double(T.M3), double(T.M4)], 2, "omitnan");
else
    pwm = nan(N,1);
    warning("PWM 컬럼을 찾지 못했습니다.");
end

roll_deg  = col_or(T, cols, "roll_deg", col_or(T, cols, "ekf_roll_deg", nan(N,1)));
pitch_deg = col_or(T, cols, "pitch_deg", col_or(T, cols, "ekf_pitch_deg", nan(N,1)));
yaw_deg   = col_or(T, cols, "yaw_deg", col_or(T, cols, "ekf_yaw_deg", nan(N,1)));

innov_pos = [ ...
    col_or(T, cols, "ekf_innov_pos_n", nan(N,1)), ...
    col_or(T, cols, "ekf_innov_pos_e", nan(N,1)), ...
    col_or(T, cols, "ekf_innov_pos_d", nan(N,1)) ...
];
innov_pos_h = hypot(innov_pos(:,1), innov_pos(:,2));

innov_vel = [ ...
    col_or(T, cols, "ekf_innov_vel_n", nan(N,1)), ...
    col_or(T, cols, "ekf_innov_vel_e", nan(N,1)), ...
    col_or(T, cols, "ekf_innov_vel_d", nan(N,1)) ...
];
innov_vel_h = hypot(innov_vel(:,1), innov_vel(:,2));

R_vel = [ ...
    col_or(T, cols, "ekf_R_applied_gnss_vel_n", nan(N,1)), ...
    col_or(T, cols, "ekf_R_applied_gnss_vel_e", nan(N,1)), ...
    col_or(T, cols, "ekf_R_applied_gnss_vel_d", nan(N,1)) ...
];
sigma_vel_h = sqrt(max(0.5 * (R_vel(:,1) + R_vel(:,2)), 0));

R_pos = [ ...
    col_or(T, cols, "ekf_R_applied_gnss_pos_n", nan(N,1)), ...
    col_or(T, cols, "ekf_R_applied_gnss_pos_e", nan(N,1)), ...
    col_or(T, cols, "ekf_R_applied_gnss_pos_d", nan(N,1)) ...
];
sigma_pos_h = sqrt(max(0.5 * (R_pos(:,1) + R_pos(:,2)), 0));

v_cov = [ ...
    col_or(T, cols, "ekf_v_cov_n", nan(N,1)), ...
    col_or(T, cols, "ekf_v_cov_e", nan(N,1)), ...
    col_or(T, cols, "ekf_v_cov_d", nan(N,1)) ...
];
sigma_P_vel_h = sqrt(max(0.5 * (v_cov(:,1) + v_cov(:,2)), 0));

p_cov = [ ...
    col_or(T, cols, "ekf_p_cov_n", nan(N,1)), ...
    col_or(T, cols, "ekf_p_cov_e", nan(N,1)), ...
    col_or(T, cols, "ekf_p_cov_d", nan(N,1)) ...
];
sigma_P_pos_h = sqrt(max(0.5 * (p_cov(:,1) + p_cov(:,2)), 0));

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

eff_gain_vel_h = delta_vel_gnss_h ./ max(innov_vel_h, 1e-9);
eff_gain_vel_h(~isfinite(eff_gain_vel_h)) = NaN;

gnss_update_executed = bool_col(T, cols, "gnss_update_executed", false(N,1));
gnss_accepted        = bool_col(T, cols, "gnss_correction_accepted", false(N,1));
gnss_velocity_used   = bool_col(T, cols, "gnss_velocity_used", false(N,1));

zupt_active          = bool_col(T, cols, "zupt_active", false(N,1));
stationary_detected  = bool_col(T, cols, "stationary_detected", false(N,1));
ekf_dt_clamped       = bool_col(T, cols, "ekf_dt_clamped", false(N,1));
tether_candidate_csv = bool_col(T, cols, "tether_disturbance_candidate", false(N,1));

vel_ctrl_valid = bool_col(T, cols, "vel_ctrl_valid", false(N,1));
vel_ctrl_enabled = bool_col(T, cols, "vel_ctrl_enabled", false(N,1));

vel_sp_n = col_or(T, cols, "vel_sp_n_mps", nan(N,1));
vel_sp_e = col_or(T, cols, "vel_sp_e_mps", nan(N,1));
vel_meas_n = col_or(T, cols, "vel_meas_n_mps", nan(N,1));
vel_meas_e = col_or(T, cols, "vel_meas_e_mps", nan(N,1));
vel_err_h = hypot(col_or(T, cols, "vel_err_n_mps", nan(N,1)), ...
                  col_or(T, cols, "vel_err_e_mps", nan(N,1)));

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

if use_vel_ctrl_valid_for_eval && any(vel_ctrl_valid)
    mask = mask & vel_ctrl_valid;
end

if sum(mask) < 5
    error("평가 가능한 샘플이 너무 적습니다. mask count = %d", sum(mask));
end

%% ============================================================
% 4. Disturbance Classification
% =============================================================

acc_disturbance = (acc_ned_h > acc_h_threshold) | (raw_acc_norm_g > raw_acc_norm_th_g);

high_pwm_acc_disturbance = (pwm > pwm_high_threshold) & ...
                           (acc_ned_h > acc_h_threshold) & ...
                           (gnss_speed_h < gnss_speed_low_th);

velocity_spike = ekf_speed_h >= speed_spike_th;

normal_mask = mask & ~acc_disturbance;
acc_dist_mask = mask & acc_disturbance;
high_pwm_dist_mask = mask & high_pwm_acc_disturbance;
spike_mask = mask & velocity_spike;

% 대표 zoom 시점:
%  1순위: high PWM acc disturbance 중 EKF speed 최대
%  2순위: acc disturbance 중 EKF speed 최대
%  3순위: 전체 mask 중 EKF speed 최대
zoom_candidates = high_pwm_dist_mask;
if ~any(zoom_candidates)
    zoom_candidates = acc_dist_mask;
end
if ~any(zoom_candidates)
    zoom_candidates = mask;
end

tmp = ekf_speed_h;
tmp(~zoom_candidates) = -inf;
[~, event_idx] = max(tmp);
event_time = t(event_idx);

zoom_mask = t >= event_time - zoom_before_sec & t <= event_time + zoom_after_sec;

%% ============================================================
% 5. Summary Statistics
% =============================================================

S = struct();
S.csv_file = string(csv_file);
S.rows = N;
S.eval_samples = sum(mask);
S.eval_t_start = min(t(mask));
S.eval_t_end = max(t(mask));

S.gnss_hacc = stat_pos(gnss_hacc(mask));
S.gnss_sacc = stat_pos(gnss_sacc(mask));

S.raw_acc_norm_g = stat_pos(raw_acc_norm_g(mask));
S.acc_body_norm = stat_pos(acc_body_norm(mask));
S.acc_ned_h = stat_pos(acc_ned_h(mask));

S.ekf_speed_h = stat_pos(ekf_speed_h(mask));
S.gnss_speed_h = stat_pos(gnss_speed_h(mask));
S.vel_diff_h = stat_pos(vel_diff_h(mask));

S.pos_err_h = stat_pos(pos_err_h(mask));
S.innov_pos_h = stat_pos(innov_pos_h(mask));
S.innov_vel_h = stat_pos(innov_vel_h(mask));

S.sigma_vel_h = stat_pos(sigma_vel_h(mask));
S.sigma_P_vel_h = stat_pos(sigma_P_vel_h(mask));
S.sigma_pos_h = stat_pos(sigma_pos_h(mask));
S.sigma_P_pos_h = stat_pos(sigma_P_pos_h(mask));

S.normal_samples = sum(normal_mask);
S.acc_dist_samples = sum(acc_dist_mask);
S.high_pwm_dist_samples = sum(high_pwm_dist_mask);
S.velocity_spike_samples = sum(spike_mask);

S.acc_dist_ratio = sum(acc_dist_mask) / sum(mask);
S.high_pwm_dist_ratio = sum(high_pwm_dist_mask) / sum(mask);
S.velocity_spike_ratio = sum(spike_mask) / sum(mask);

S.gnss_update_executed_count = sum(gnss_update_executed & mask);
S.gnss_accepted_count = sum(gnss_accepted & mask);
S.gnss_velocity_used_count = sum(gnss_velocity_used & mask);
S.zupt_active_count = sum(zupt_active & mask);
S.dt_clamped_count = sum(ekf_dt_clamped & mask);

S.event_time = event_time;
S.event_pwm = pwm(event_idx);
S.event_ekf_speed_h = ekf_speed_h(event_idx);
S.event_gnss_speed_h = gnss_speed_h(event_idx);
S.event_vel_diff_h = vel_diff_h(event_idx);
S.event_raw_acc_norm_g = raw_acc_norm_g(event_idx);
S.event_acc_body_norm = acc_body_norm(event_idx);
S.event_acc_ned_h = acc_ned_h(event_idx);
S.event_acc_ned_n = acc_ned(event_idx,1);
S.event_acc_ned_e = acc_ned(event_idx,2);
S.event_acc_ned_d = acc_ned(event_idx,3);
S.event_roll_deg = roll_deg(event_idx);
S.event_pitch_deg = pitch_deg(event_idx);
S.event_yaw_deg = yaw_deg(event_idx);
S.event_innov_vel_h = innov_vel_h(event_idx);
S.event_delta_vel_gnss_h = delta_vel_gnss_h(event_idx);
S.event_delta_vel_zupt_h = delta_vel_zupt_h(event_idx);
S.event_eff_gain_vel_h = eff_gain_vel_h(event_idx);
S.event_acc_disturbance = acc_disturbance(event_idx);
S.event_high_pwm_acc_disturbance = high_pwm_acc_disturbance(event_idx);
S.event_tether_candidate_csv = tether_candidate_csv(event_idx);

%% ============================================================
% 6. Console Report
% =============================================================

fprintf("\n=================================================\n");
fprintf("Evaluation Summary\n");
fprintf("=================================================\n");
fprintf("Evaluation samples              : %d\n", S.eval_samples);
fprintf("Evaluation time range           : %.3f ~ %.3f sec\n", S.eval_t_start, S.eval_t_end);
fprintf("Normal samples                  : %d\n", S.normal_samples);
fprintf("acc_disturbance samples         : %d (%.2f %%)\n", S.acc_dist_samples, 100*S.acc_dist_ratio);
fprintf("high_pwm_acc_disturbance samples: %d (%.2f %%)\n", S.high_pwm_dist_samples, 100*S.high_pwm_dist_ratio);
fprintf("velocity spike samples          : %d (%.2f %%)\n", S.velocity_spike_samples, 100*S.velocity_spike_ratio);
fprintf("GNSS hAcc mean/median/max       : %.4f / %.4f / %.4f m\n", S.gnss_hacc.mean, S.gnss_hacc.median, S.gnss_hacc.max);
fprintf("GNSS sAcc mean/median/max       : %.4f / %.4f / %.4f m/s\n", S.gnss_sacc.mean, S.gnss_sacc.median, S.gnss_sacc.max);
fprintf("GNSS update executed / accepted : %d / %d\n", S.gnss_update_executed_count, S.gnss_accepted_count);
fprintf("GNSS velocity used count        : %d\n", S.gnss_velocity_used_count);
fprintf("ZUPT active count               : %d\n", S.zupt_active_count);
fprintf("dt clamped count                : %d\n", S.dt_clamped_count);

print_line_stats("raw_acc_norm_g", S.raw_acc_norm_g, "g");
print_line_stats("acc_body_norm", S.acc_body_norm, "m/s^2");
print_line_stats("acc_ned_h", S.acc_ned_h, "m/s^2");
print_line_stats("ekf_speed_h", S.ekf_speed_h, "m/s");
print_line_stats("gnss_speed_h", S.gnss_speed_h, "m/s");
print_line_stats("vel_diff_h", S.vel_diff_h, "m/s");
print_line_stats("pos_err_h", S.pos_err_h, "m");

fprintf("\n=================================================\n");
fprintf("Representative Event / Zoom Point\n");
fprintf("=================================================\n");
fprintf("time                             : %.3f sec\n", S.event_time);
fprintf("PWM                              : %.2f\n", S.event_pwm);
fprintf("EKF speed H                      : %.4f m/s\n", S.event_ekf_speed_h);
fprintf("GNSS speed H                     : %.4f m/s\n", S.event_gnss_speed_h);
fprintf("GNSS-EKF velocity diff H         : %.4f m/s\n", S.event_vel_diff_h);
fprintf("raw accel norm                   : %.4f g\n", S.event_raw_acc_norm_g);
fprintf("ekf_accel_body norm              : %.4f m/s^2\n", S.event_acc_body_norm);
fprintf("acc_ned H                        : %.4f m/s^2\n", S.event_acc_ned_h);
fprintf("acc_ned N/E/D                    : %.4f / %.4f / %.4f m/s^2\n", S.event_acc_ned_n, S.event_acc_ned_e, S.event_acc_ned_d);
fprintf("roll/pitch/yaw                   : %.3f / %.3f / %.3f deg\n", S.event_roll_deg, S.event_pitch_deg, S.event_yaw_deg);
fprintf("innov_vel_H                      : %.4f m/s\n", S.event_innov_vel_h);
fprintf("dV_GNSS_H / dV_ZUPT_H            : %.4f / %.4f m/s\n", S.event_delta_vel_gnss_h, S.event_delta_vel_zupt_h);
fprintf("effective gain approx            : %.4f\n", S.event_eff_gain_vel_h);
fprintf("acc_disturbance                  : %d\n", S.event_acc_disturbance);
fprintf("high_pwm_acc_disturbance         : %d\n", S.event_high_pwm_acc_disturbance);
fprintf("tether_candidate_csv             : %d\n", S.event_tether_candidate_csv);
fprintf("=================================================\n\n");

%% ============================================================
% 7. Make Report Table
% =============================================================

metric_name = strings(0,1);
value = [];
unit = strings(0,1);

[metric_name, value, unit] = add_metric(metric_name, value, unit, "eval_samples", S.eval_samples, "samples");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "eval_t_start", S.eval_t_start, "s");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "eval_t_end", S.eval_t_end, "s");

[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "raw_acc_norm_g", S.raw_acc_norm_g, "g");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "acc_body_norm", S.acc_body_norm, "m/s^2");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "acc_ned_h", S.acc_ned_h, "m/s^2");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "ekf_speed_h", S.ekf_speed_h, "m/s");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "gnss_speed_h", S.gnss_speed_h, "m/s");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "vel_diff_h", S.vel_diff_h, "m/s");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "pos_err_h", S.pos_err_h, "m");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "gnss_hacc", S.gnss_hacc, "m");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "gnss_sacc", S.gnss_sacc, "m/s");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "sigma_vel_h", S.sigma_vel_h, "m/s");
[metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, "sigma_P_vel_h", S.sigma_P_vel_h, "m/s");

[metric_name, value, unit] = add_metric(metric_name, value, unit, "acc_dist_samples", S.acc_dist_samples, "samples");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "acc_dist_ratio", 100*S.acc_dist_ratio, "%");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "high_pwm_dist_samples", S.high_pwm_dist_samples, "samples");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "high_pwm_dist_ratio", 100*S.high_pwm_dist_ratio, "%");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "velocity_spike_samples", S.velocity_spike_samples, "samples");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "velocity_spike_ratio", 100*S.velocity_spike_ratio, "%");

[metric_name, value, unit] = add_metric(metric_name, value, unit, "event_time", S.event_time, "s");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "event_pwm", S.event_pwm, "pwm");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "event_ekf_speed_h", S.event_ekf_speed_h, "m/s");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "event_raw_acc_norm_g", S.event_raw_acc_norm_g, "g");
[metric_name, value, unit] = add_metric(metric_name, value, unit, "event_acc_ned_h", S.event_acc_ned_h, "m/s^2");

report_table = table(metric_name(:), value(:), unit(:), ...
    'VariableNames', {'metric', 'value', 'unit'});

if save_report
    csv_report_path = fullfile(out_dir, csv_base + "_final_analysis_report.csv");
    mat_report_path = fullfile(out_dir, csv_base + "_final_analysis_report.mat");

    writetable(report_table, csv_report_path);
    save(mat_report_path, "S", "report_table", ...
        "t", "mask", "normal_mask", "acc_dist_mask", "high_pwm_dist_mask", ...
        "raw_acc_norm_g", "acc_body_norm", "acc_ned_h", ...
        "ekf_speed_h", "gnss_speed_h", "vel_diff_h", "pos_err_h", ...
        "pwm", "gnss_hacc", "gnss_sacc", "sigma_vel_h", "sigma_P_vel_h");

    fprintf("Saved report CSV: %s\n", csv_report_path);
    fprintf("Saved report MAT: %s\n", mat_report_path);
end

%% ============================================================
% 8. Figures
% =============================================================

if ~show_figures
    set(0, "DefaultFigureVisible", "off");
end

% ------------------------------------------------------------
% Figure 1. 핵심 요약: PWM -> accel -> velocity -> flags
% ------------------------------------------------------------
fig1 = figure("Name", "01 Core Overview");
set(fig1, "Color", "w");

subplot(6,1,1); hold on; grid on;
plot(t, pwm, "LineWidth", 1.1);
xline(event_time, ":");
ylabel("PWM");
title("PWM");

subplot(6,1,2); hold on; grid on;
plot(t, raw_acc_norm_g, "LineWidth", 1.0);
yline(1.0, "--");
yline(raw_acc_norm_th_g, ":");
xline(event_time, ":");
ylabel("|raw| [g]");
title("Raw accel norm before LPF/bias removal");

subplot(6,1,3); hold on; grid on;
plot(t, acc_body_norm, "LineWidth", 1.0);
xline(event_time, ":");
ylabel("|body|");
title("ekf\_accel\_body norm after LPF/bias removal [m/s^2]");

subplot(6,1,4); hold on; grid on;
plot(t, acc_ned_h, "LineWidth", 1.0);
yline(acc_h_threshold, ":");
xline(event_time, ":");
ylabel("acc H");
title("acc\_ned horizontal, actual propagation input [m/s^2]");

subplot(6,1,5); hold on; grid on;
plot(t, ekf_speed_h, "LineWidth", 1.0);
plot(t, gnss_speed_h, "--", "LineWidth", 1.0);
yline(speed_spike_th, ":");
xline(event_time, ":");
ylabel("speed H");
title("EKF speed vs GNSS speed [m/s]");
legend("EKF", "GNSS", "speed spike threshold", "event", "Location", "best");

subplot(6,1,6); hold on; grid on;
stairs(t, double(acc_disturbance), "LineWidth", 1.0);
stairs(t, double(high_pwm_acc_disturbance), "LineWidth", 1.0);
stairs(t, double(gnss_update_executed), "LineWidth", 0.8);
stairs(t, double(zupt_active), "LineWidth", 1.0);
xline(event_time, ":");
ylim([-0.1, 1.1]);
ylabel("flag");
xlabel("Time [s]");
title("Disturbance / GNSS update / ZUPT");
legend("acc disturbance", "high PWM acc disturbance", "GNSS update", "ZUPT", "Location", "best");

save_fig_if_needed(fig1, out_dir, csv_base + "_fig01_core_overview.png", save_figures);

% ------------------------------------------------------------
% Figure 2. 대표 event zoom
% ------------------------------------------------------------
fig2 = figure("Name", "02 Event Zoom");
set(fig2, "Color", "w");

subplot(7,1,1); hold on; grid on;
plot(t(zoom_mask), pwm(zoom_mask), "LineWidth", 1.1);
xline(event_time, ":");
ylabel("PWM");
title(sprintf("Event zoom around %.3f sec", event_time));

subplot(7,1,2); hold on; grid on;
plot(t(zoom_mask), raw_acc_norm_g(zoom_mask), "LineWidth", 1.0);
yline(1.0, "--");
yline(raw_acc_norm_th_g, ":");
xline(event_time, ":");
ylabel("|raw| [g]");
title("Raw accel norm");

subplot(7,1,3); hold on; grid on;
plot(t(zoom_mask), acc_ned(zoom_mask,1), "LineWidth", 1.0);
plot(t(zoom_mask), acc_ned(zoom_mask,2), "LineWidth", 1.0);
plot(t(zoom_mask), acc_ned_h(zoom_mask), "LineWidth", 1.1);
yline(acc_h_threshold, ":");
xline(event_time, ":");
ylabel("acc");
title("acc\_ned N/E/H");
legend("N", "E", "H", "threshold", "event", "Location", "best");

subplot(7,1,4); hold on; grid on;
plot(t(zoom_mask), roll_deg(zoom_mask), "LineWidth", 1.0);
plot(t(zoom_mask), pitch_deg(zoom_mask), "LineWidth", 1.0);
xline(event_time, ":");
ylabel("[deg]");
title("Roll / Pitch");
legend("roll", "pitch", "event", "Location", "best");

subplot(7,1,5); hold on; grid on;
plot(t(zoom_mask), ekf_speed_h(zoom_mask), "LineWidth", 1.1);
plot(t(zoom_mask), gnss_speed_h(zoom_mask), "--", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("speed");
title("EKF/GNSS horizontal speed");
legend("EKF", "GNSS", "event", "Location", "best");

subplot(7,1,6); hold on; grid on;
plot(t(zoom_mask), innov_vel_h(zoom_mask), "LineWidth", 1.0);
plot(t(zoom_mask), delta_vel_gnss_h(zoom_mask), "LineWidth", 1.0);
plot(t(zoom_mask), delta_vel_zupt_h(zoom_mask), "LineWidth", 1.0);
plot(t(zoom_mask), sigma_vel_h(zoom_mask), "--", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("[m/s]");
title("Velocity correction diagnostic");
legend("innov vel H", "dV GNSS H", "dV ZUPT H", "sigma R vel H", "event", "Location", "best");

subplot(7,1,7); hold on; grid on;
stairs(t(zoom_mask), double(gnss_update_executed(zoom_mask)), "LineWidth", 1.0);
stairs(t(zoom_mask), double(gnss_accepted(zoom_mask)), "LineWidth", 1.0);
stairs(t(zoom_mask), double(gnss_velocity_used(zoom_mask)), "LineWidth", 1.0);
stairs(t(zoom_mask), double(zupt_active(zoom_mask)), "LineWidth", 1.0);
xline(event_time, ":");
ylim([-0.1, 1.1]);
ylabel("flag");
xlabel("Time [s]");
title("Correction flags");
legend("GNSS update", "GNSS accepted", "GNSS vel used", "ZUPT", "event", "Location", "best");

save_fig_if_needed(fig2, out_dir, csv_base + "_fig02_event_zoom.png", save_figures);

% ------------------------------------------------------------
% Figure 3. 분포: damping 효과를 설명하기 좋은 핵심 histogram
% ------------------------------------------------------------
fig3 = figure("Name", "03 Distributions");
set(fig3, "Color", "w");

subplot(4,1,1); hold on; grid on;
histogram(raw_acc_norm_g(mask), 40);
xline(raw_acc_norm_th_g, ":");
xlabel("raw accel norm [g]");
ylabel("count");
title("Raw accel norm distribution");

subplot(4,1,2); hold on; grid on;
histogram(acc_ned_h(mask), 40);
xline(acc_h_threshold, ":");
xlabel("acc\_ned H [m/s^2]");
ylabel("count");
title("Propagation horizontal acceleration distribution");

subplot(4,1,3); hold on; grid on;
histogram(ekf_speed_h(mask), 40);
xline(speed_spike_th, ":");
xlabel("EKF speed H [m/s]");
ylabel("count");
title("EKF horizontal speed distribution");

subplot(4,1,4); hold on; grid on;
histogram(pos_err_h(mask), 40);
xlabel("Horizontal position error [m]");
ylabel("count");
title("Horizontal position error distribution");

save_fig_if_needed(fig3, out_dir, csv_base + "_fig03_distributions.png", save_figures);

% ------------------------------------------------------------
% Figure 4. Normal vs disturbance: 같은 파일 내부에서 원인 분리
% ------------------------------------------------------------
fig4 = figure("Name", "04 Normal vs Disturbance");
set(fig4, "Color", "w");

subplot(4,1,1); hold on; grid on;
histogram(ekf_speed_h(normal_mask), 30);
histogram(ekf_speed_h(acc_dist_mask), 30);
xlabel("EKF speed H [m/s]");
ylabel("count");
title("EKF speed: normal vs acc disturbance");
legend("normal", "acc disturbance");

subplot(4,1,2); hold on; grid on;
histogram(raw_acc_norm_g(normal_mask), 30);
histogram(raw_acc_norm_g(acc_dist_mask), 30);
xlabel("raw accel norm [g]");
ylabel("count");
title("Raw accel norm: normal vs acc disturbance");
legend("normal", "acc disturbance");

subplot(4,1,3); hold on; grid on;
histogram(acc_ned_h(normal_mask), 30);
histogram(acc_ned_h(acc_dist_mask), 30);
xlabel("acc\_ned H [m/s^2]");
ylabel("count");
title("acc\_ned H: normal vs acc disturbance");
legend("normal", "acc disturbance");

subplot(4,1,4); hold on; grid on;
plot(t(mask), pos_err_h(mask), "LineWidth", 1.0);
plot(t(mask), gnss_hacc(mask), "--", "LineWidth", 1.0);
xline(event_time, ":");
xlabel("Time [s]");
ylabel("[m]");
title("Position error vs GNSS hAcc");
legend("H error", "hAcc", "event", "Location", "best");

save_fig_if_needed(fig4, out_dir, csv_base + "_fig04_normal_vs_disturbance.png", save_figures);

% ------------------------------------------------------------
% Figure 5. GNSS velocity update / covariance sanity
% ------------------------------------------------------------
fig5 = figure("Name", "05 GNSS Velocity Update");
set(fig5, "Color", "w");

subplot(5,1,1); hold on; grid on;
plot(t, ekf_vel(:,1), "LineWidth", 1.0);
plot(t, gnss_vel(:,1), "--", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("V_N");
title("Velocity N [m/s]");
legend("EKF", "GNSS", "event", "Location", "best");

subplot(5,1,2); hold on; grid on;
plot(t, ekf_vel(:,2), "LineWidth", 1.0);
plot(t, gnss_vel(:,2), "--", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("V_E");
title("Velocity E [m/s]");
legend("EKF", "GNSS", "event", "Location", "best");

subplot(5,1,3); hold on; grid on;
plot(t, innov_vel_h, "LineWidth", 1.0);
plot(t, sigma_vel_h, "--", "LineWidth", 1.0);
plot(t, gnss_sacc, ":", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("[m/s]");
title("Velocity innovation H vs applied sigma and GNSS sAcc");
legend("innov vel H", "sigma R vel H", "sAcc", "event", "Location", "best");

subplot(5,1,4); hold on; grid on;
plot(t, delta_vel_gnss_h, "LineWidth", 1.0);
plot(t, delta_vel_zupt_h, "LineWidth", 1.0);
xline(event_time, ":");
ylabel("[m/s]");
title("Correction delta velocity");
legend("dV GNSS H", "dV ZUPT H", "event", "Location", "best");

subplot(5,1,5); hold on; grid on;
plot(t, sigma_P_vel_h, "LineWidth", 1.0);
plot(t, sigma_vel_h, "--", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("[m/s]");
xlabel("Time [s]");
title("sqrt(P\_vel\_H) vs sigma R vel H");
legend("sqrt(P vel H)", "sigma R vel H", "event", "Location", "best");

save_fig_if_needed(fig5, out_dir, csv_base + "_fig05_gnss_velocity_update.png", save_figures);

% ------------------------------------------------------------
% Figure 6. Position / trajectory: 최종 성능 설명용
% ------------------------------------------------------------
fig6 = figure("Name", "06 Position Performance");
set(fig6, "Color", "w");

subplot(3,1,1); hold on; grid on; axis equal;
plot(ekf_pos(mask,2), ekf_pos(mask,1), "LineWidth", 1.2);
plot(gnss_pos(mask,2), gnss_pos(mask,1), ".", "MarkerSize", 7);
xlabel("East [m]");
ylabel("North [m]");
title("2D trajectory: EKF vs GNSS");
legend("EKF", "GNSS", "Location", "best");

subplot(3,1,2); hold on; grid on;
plot(t(mask), pos_err(mask,1), "LineWidth", 1.0);
plot(t(mask), pos_err(mask,2), "LineWidth", 1.0);
plot(t(mask), gnss_hacc(mask), "--", "LineWidth", 1.0);
plot(t(mask), -gnss_hacc(mask), "--", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("[m]");
title("N/E position error with ±hAcc reference");
legend("N error", "E error", "+hAcc", "-hAcc", "event", "Location", "best");

subplot(3,1,3); hold on; grid on;
plot(t(mask), pos_err_h(mask), "LineWidth", 1.1);
plot(t(mask), gnss_hacc(mask), "--", "LineWidth", 1.0);
plot(t(mask), sigma_P_pos_h(mask), ":", "LineWidth", 1.0);
xline(event_time, ":");
ylabel("[m]");
xlabel("Time [s]");
title("Horizontal position error");
legend("H error", "GNSS hAcc", "sqrt(P pos H)", "event", "Location", "best");

save_fig_if_needed(fig6, out_dir, csv_base + "_fig06_position_performance.png", save_figures);

% ------------------------------------------------------------
% Figure 7. Velocity controller optional
% ------------------------------------------------------------
if any(isfinite(vel_sp_n)) && any(isfinite(vel_meas_n))
    fig7 = figure("Name", "07 Velocity Controller Optional");
    set(fig7, "Color", "w");

    subplot(4,1,1); hold on; grid on;
    plot(t, pwm, "LineWidth", 1.0);
    xline(event_time, ":");
    ylabel("PWM");
    title("PWM");

    subplot(4,1,2); hold on; grid on;
    plot(t, vel_sp_n, "LineWidth", 1.0);
    plot(t, vel_meas_n, "--", "LineWidth", 1.0);
    xline(event_time, ":");
    ylabel("N [m/s]");
    title("Velocity controller N");
    legend("SP", "meas", "event", "Location", "best");

    subplot(4,1,3); hold on; grid on;
    plot(t, vel_sp_e, "LineWidth", 1.0);
    plot(t, vel_meas_e, "--", "LineWidth", 1.0);
    xline(event_time, ":");
    ylabel("E [m/s]");
    title("Velocity controller E");
    legend("SP", "meas", "event", "Location", "best");

    subplot(4,1,4); hold on; grid on;
    plot(t, vel_err_h, "LineWidth", 1.0);
    stairs(t, double(vel_ctrl_valid), "LineWidth", 1.0);
    xline(event_time, ":");
    ylabel("err / flag");
    xlabel("Time [s]");
    title("Velocity error H and vel\_ctrl\_valid");
    legend("vel error H", "vel ctrl valid", "event", "Location", "best");

    save_fig_if_needed(fig7, out_dir, csv_base + "_fig07_velocity_controller.png", save_figures);
end

fprintf("\nDone.\n");
fprintf("대표적으로 볼 figure:\n");
fprintf("  01 Core Overview\n");
fprintf("  02 Event Zoom\n");
fprintf("  03 Distributions\n");
fprintf("  04 Normal vs Disturbance\n");
fprintf("  05 GNSS Velocity Update\n");
fprintf("  06 Position Performance\n\n");

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

function [metric_name, value, unit] = add_metric(metric_name, value, unit, name, val, unit_str)
    metric_name(end+1,1) = string(name);
    value(end+1,1) = val;
    unit(end+1,1) = string(unit_str);
end

function [metric_name, value, unit] = add_stats_to_table(metric_name, value, unit, prefix, s, unit_str)
    fields = ["mean","median","std","rms","p95","p99","min","max"];
    for k = 1:numel(fields)
        f = fields(k);
        [metric_name, value, unit] = add_metric(metric_name, value, unit, ...
            prefix + "_" + f, s.(f), unit_str);
    end
end

function save_fig_if_needed(fig_handle, out_dir, filename, save_figures)
    if save_figures
        exportgraphics(fig_handle, fullfile(out_dir, filename), "Resolution", 180);
    end
end
