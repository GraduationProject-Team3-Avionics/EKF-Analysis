clear; clc; close all;

%% ============================================================
%  EKF Performance Evaluation From CSV
%  - GNSS hAcc/vAcc + Applied R + Normalized Innovation 포함 버전
% =============================================================
% 목적:
%   1) EKF position을 GNSS local NED 기준으로 평가
%   2) GNSS가 제공한 hAcc/vAcc와 EKF-GNSS 오차 비교
%   3) EKF에서 실제 적용한 GNSS R 값 확인
%   4) Innovation을 R 기준으로 정규화하여 R 설정이 과한지/약한지 판단
%
% 현재 TEST_03.CSV 기준 추가 반영 컬럼:
%   gnss_hacc_m
%   gnss_vacc_m
%   ekf_R_applied_gnss_pos_n/e/d
%   gnss_correction_accepted
%   gnss_innov_gate_reject_count

%% ============================================================
%  0. User Settings
% =============================================================

% CSV 경로
% csv_file = "260610\fc_damp_03.CSV";
csv_file = "data\vel_test_03.CSV";

% 평가 구간 [s]
eval_start_sec = 0.0;
eval_end_sec   = inf;

% D축을 최종 3D 성능 평가에 포함할지 여부
% 현재는 N/E 수평 중심 평가이므로 false 추천
use_vertical_axis = false;

% GNSS valid 조건
min_fix_type = 3;
min_num_sats = 6;

% gnss_ref_ready 컬럼이 있으면 reference 준비 완료 구간만 평가할지
use_gnss_ref_ready = true;

% ekf_ready 컬럼이 있으면 EKF ready 구간만 평가할지
use_ekf_ready = true;

% 초기 0값 제거
remove_zero_gnss = true;
remove_zero_ekf  = false;

% outlier 제거 옵션
remove_outlier = false;
horizontal_error_gate_m = 10.0;

% 정지 상태 판단용 속도 threshold
stationary_speed_threshold_mps = 0.15;

% plot에서 D축 관련 figure를 그릴지
plot_vertical_axis = false;

% GNSS accuracy / R plot 옵션
plot_gnss_accuracy = true;
plot_applied_R     = true;
plot_normalized_innovation = true;

% Normalized innovation에서 참고선
norm_gate_2 = 2.0;
norm_gate_3 = 3.0;

%% ============================================================
%  1. CSV Load
% =============================================================
if ~isfile(csv_file)
    error("CSV 파일을 찾을 수 없습니다: %s", csv_file);
end

T = readtable(csv_file, "VariableNamingRule", "preserve");
cols = string(T.Properties.VariableNames);

fprintf("\n=================================================\n");
fprintf("EKF Performance Evaluation From CSV\n");
fprintf("=================================================\n");
fprintf("Loaded CSV : %s\n", csv_file);
fprintf("Rows       : %d\n", height(T));
fprintf("Columns    : %d\n", width(T));

%% ============================================================
%  2. Time Vector
% =============================================================
require_cols(cols, "timestamp_ms");

t = double(T.timestamp_ms) * 1.0e-3;
t = t - t(1);

N = height(T);

%% ============================================================
%  3. EKF Position / Velocity Load
% =============================================================
required_ekf_pos = ["ekf_pos_n", "ekf_pos_e", "ekf_pos_d"];
required_ekf_vel = ["ekf_vel_n", "ekf_vel_e", "ekf_vel_d"];

require_cols(cols, required_ekf_pos);
require_cols(cols, required_ekf_vel);

ekf_pos = [ ...
    double(T.ekf_pos_n), ...
    double(T.ekf_pos_e), ...
    double(T.ekf_pos_d) ...
];

ekf_vel = [ ...
    double(T.ekf_vel_n), ...
    double(T.ekf_vel_e), ...
    double(T.ekf_vel_d) ...
];

%% ============================================================
%  4. GNSS Position Load
% =============================================================
has_ekf_gnss_pos = all(ismember(["ekf_gnss_pos_n", "ekf_gnss_pos_e", "ekf_gnss_pos_d"], cols));

if has_ekf_gnss_pos
    fprintf("GNSS reference source: ekf_gnss_pos_n/e/d\n");

    gnss_pos = [ ...
        double(T.ekf_gnss_pos_n), ...
        double(T.ekf_gnss_pos_e), ...
        double(T.ekf_gnss_pos_d) ...
    ];

else
    fprintf("GNSS reference source: lat/lon/hmsl -> local NED\n");

    required_gnss = ["lat", "lon", "hmsl"];
    require_cols(cols, required_gnss);

    lat = double(T.lat);
    lon = double(T.lon);
    hmsl = double(T.hmsl);

    valid_ref = isfinite(lat) & isfinite(lon) & isfinite(hmsl) & ...
                abs(lat) > 1.0e-12 & abs(lon) > 1.0e-12;

    ref_idx = find(valid_ref, 1, "first");

    if isempty(ref_idx)
        error("유효한 GNSS 기준점을 찾을 수 없습니다.");
    end

    ref_lat = deg2rad(lat(ref_idx));
    ref_lon = deg2rad(lon(ref_idx));
    ref_h   = hmsl(ref_idx);

    gnss_pos = nan(N, 3);

    for i = 1:N
        if valid_ref(i)
            gnss_pos(i, :) = lla_to_local_ned( ...
                deg2rad(lat(i)), deg2rad(lon(i)), hmsl(i), ...
                ref_lat, ref_lon, ref_h).';
        end
    end
end

%% ============================================================
%  5. GNSS Accuracy / Applied R Load
% =============================================================

has_gnss_hacc = ismember("gnss_hacc_m", cols);
has_gnss_vacc = ismember("gnss_vacc_m", cols);

if has_gnss_hacc
    gnss_hacc_m = double(T.gnss_hacc_m);
else
    gnss_hacc_m = nan(N, 1);
    fprintf("Warning: gnss_hacc_m 컬럼이 없습니다.\n");
end

if has_gnss_vacc
    gnss_vacc_m = double(T.gnss_vacc_m);
else
    gnss_vacc_m = nan(N, 1);
    fprintf("Warning: gnss_vacc_m 컬럼이 없습니다.\n");
end

has_applied_R = all(ismember([ ...
    "ekf_R_applied_gnss_pos_n", ...
    "ekf_R_applied_gnss_pos_e", ...
    "ekf_R_applied_gnss_pos_d"], cols));

if has_applied_R
    R_applied = [ ...
        double(T.ekf_R_applied_gnss_pos_n), ...
        double(T.ekf_R_applied_gnss_pos_e), ...
        double(T.ekf_R_applied_gnss_pos_d) ...
    ];

    sigma_R_applied = sqrt(max(R_applied, 0));
else
    R_applied = nan(N, 3);
    sigma_R_applied = nan(N, 3);
    fprintf("Warning: ekf_R_applied_gnss_pos_n/e/d 컬럼이 없습니다.\n");
end

has_correction_accepted = ismember("gnss_correction_accepted", cols);
if has_correction_accepted
    gnss_correction_accepted = double(T.gnss_correction_accepted) ~= 0;
else
    gnss_correction_accepted = false(N, 1);
end

has_reject_count = ismember("gnss_innov_gate_reject_count", cols);
if has_reject_count
    gnss_reject_count = double(T.gnss_innov_gate_reject_count);
else
    gnss_reject_count = nan(N, 1);
end

%% ============================================================
%  6. GNSS Valid Mask
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

if remove_zero_gnss
    gnss_valid = gnss_valid & vecnorm(gnss_pos, 2, 2) > 1.0e-9;
end

%% ============================================================
%  7. Evaluation Mask
% =============================================================
eval_mask = t >= eval_start_sec & t <= eval_end_sec;

valid_ekf = all(isfinite(ekf_pos), 2);

if use_ekf_ready && ismember("ekf_ready", cols)
    valid_ekf = valid_ekf & double(T.ekf_ready) ~= 0;
end

if remove_zero_ekf
    valid_ekf = valid_ekf & vecnorm(ekf_pos, 2, 2) > 1.0e-9;
end

mask = eval_mask & valid_ekf & gnss_valid;

pos_error_all = gnss_pos - ekf_pos;
horizontal_error_all = sqrt(pos_error_all(:, 1).^2 + pos_error_all(:, 2).^2);

if remove_outlier
    mask = mask & horizontal_error_all < horizontal_error_gate_m;
end

num_eval = sum(mask);

if num_eval < 5
    error("평가 가능한 샘플이 너무 적습니다. num_eval = %d", num_eval);
end

first_idx = find(mask, 1, "first");
last_idx  = find(mask, 1, "last");

fprintf("Evaluation samples: %d\n", num_eval);
fprintf("Time range        : %.3f ~ %.3f sec\n", t(first_idx), t(last_idx));
fprintf("Use vertical axis : %d\n", use_vertical_axis);
fprintf("Use gnss_ref_ready: %d\n", use_gnss_ref_ready);
fprintf("Use ekf_ready     : %d\n", use_ekf_ready);
fprintf("=================================================\n\n");

%% ============================================================
%  8. Position Error
% =============================================================
% error = GNSS - EKF
pos_error = gnss_pos(mask, :) - ekf_pos(mask, :);

err_N = pos_error(:, 1);
err_E = pos_error(:, 2);
err_D = pos_error(:, 3);

horizontal_error = sqrt(err_N.^2 + err_E.^2);

if use_vertical_axis
    three_d_error = sqrt(err_N.^2 + err_E.^2 + err_D.^2);
else
    three_d_error = nan(size(horizontal_error));
end

%% ============================================================
%  9. Innovation Load
% =============================================================
has_innov = all(ismember(["ekf_innov_pos_n", "ekf_innov_pos_e", "ekf_innov_pos_d"], cols));

if has_innov
    innov_pos = [ ...
        double(T.ekf_innov_pos_n), ...
        double(T.ekf_innov_pos_e), ...
        double(T.ekf_innov_pos_d) ...
    ];

    innov_valid = eval_mask & all(isfinite(innov_pos), 2);

    if use_ekf_ready && ismember("ekf_ready", cols)
        innov_valid = innov_valid & double(T.ekf_ready) ~= 0;
    end

    if use_gnss_ref_ready && ismember("gnss_ref_ready", cols)
        innov_valid = innov_valid & double(T.gnss_ref_ready) ~= 0;
    end

    % innovation이 전부 0인 초기 구간은 제거
    innov_valid = innov_valid & vecnorm(innov_pos, 2, 2) > 1.0e-12;

    innov_N = innov_pos(innov_valid, 1);
    innov_E = innov_pos(innov_valid, 2);
    innov_D = innov_pos(innov_valid, 3);

    innov_horizontal = sqrt(innov_N.^2 + innov_E.^2);

else
    innov_pos = nan(N, 3);
    innov_valid = false(N, 1);

    innov_N = [];
    innov_E = [];
    innov_D = [];
    innov_horizontal = [];

    fprintf("Warning: ekf_innov_pos_n/e/d 컬럼이 없어 innovation 평가는 생략합니다.\n");
end

%% ============================================================
%  10. R-normalized Innovation
% =============================================================
% 엄밀한 NIS는 S = HPH' + R 를 사용해야 함.
% 여기서는 CSV에 S가 없으므로, 적용 R만 기준으로 대략적인 정규화 지표를 봄.
%   norm_innov_N = innov_N / sqrt(R_N)
%   norm_innov_E = innov_E / sqrt(R_E)
%   norm_innov_NE_rss = sqrt(innov_N^2/R_N + innov_E^2/R_E)

if has_innov && has_applied_R
    norm_valid = innov_valid & ...
                 all(isfinite(R_applied), 2) & ...
                 R_applied(:, 1) > 1.0e-12 & ...
                 R_applied(:, 2) > 1.0e-12 & ...
                 R_applied(:, 3) > 1.0e-12;

    norm_innov_N = innov_pos(norm_valid, 1) ./ sqrt(R_applied(norm_valid, 1));
    norm_innov_E = innov_pos(norm_valid, 2) ./ sqrt(R_applied(norm_valid, 2));
    norm_innov_D = innov_pos(norm_valid, 3) ./ sqrt(R_applied(norm_valid, 3));

    norm_innov_NE_rss = sqrt( ...
        innov_pos(norm_valid, 1).^2 ./ R_applied(norm_valid, 1) + ...
        innov_pos(norm_valid, 2).^2 ./ R_applied(norm_valid, 2) );

else
    norm_valid = false(N, 1);
    norm_innov_N = [];
    norm_innov_E = [];
    norm_innov_D = [];
    norm_innov_NE_rss = [];
end

%% ============================================================
%  11. Velocity Stats
% =============================================================
vel_mask = eval_mask & all(isfinite(ekf_vel), 2);

if use_ekf_ready && ismember("ekf_ready", cols)
    vel_mask = vel_mask & double(T.ekf_ready) ~= 0;
end

vel_N = ekf_vel(vel_mask, 1);
vel_E = ekf_vel(vel_mask, 2);
vel_D = ekf_vel(vel_mask, 3);

horizontal_speed = sqrt(vel_N.^2 + vel_E.^2);

% 정지 상태 velocity 평가
stationary_mask = vel_mask;
stationary_speed_all = sqrt(ekf_vel(:, 1).^2 + ekf_vel(:, 2).^2);
stationary_mask = stationary_mask & stationary_speed_all <= stationary_speed_threshold_mps;

stationary_vel_N = ekf_vel(stationary_mask, 1);
stationary_vel_E = ekf_vel(stationary_mask, 2);
stationary_vel_D = ekf_vel(stationary_mask, 3);
stationary_speed = sqrt(stationary_vel_N.^2 + stationary_vel_E.^2);

%% ============================================================
%  12. Statistics
% =============================================================
stats = struct();

stats.csv_file = csv_file;
stats.num_eval_samples = num_eval;
stats.eval_start_sec = t(first_idx);
stats.eval_end_sec = t(last_idx);

stats.pos_N = calc_scalar_stats(err_N);
stats.pos_E = calc_scalar_stats(err_E);
stats.pos_D = calc_scalar_stats(err_D);

stats.horizontal = calc_positive_stats(horizontal_error);

if use_vertical_axis
    stats.three_d = calc_positive_stats(three_d_error);
else
    stats.three_d = empty_stats();
end

if has_innov
    stats.innov_N = calc_scalar_stats(innov_N);
    stats.innov_E = calc_scalar_stats(innov_E);
    stats.innov_D = calc_scalar_stats(innov_D);
    stats.innov_horizontal = calc_positive_stats(innov_horizontal);
else
    stats.innov_N = empty_stats();
    stats.innov_E = empty_stats();
    stats.innov_D = empty_stats();
    stats.innov_horizontal = empty_stats();
end

if has_gnss_hacc
    stats.gnss_hacc_m = calc_positive_stats(gnss_hacc_m(mask));
else
    stats.gnss_hacc_m = empty_stats();
end

if has_gnss_vacc
    stats.gnss_vacc_m = calc_positive_stats(gnss_vacc_m(mask));
else
    stats.gnss_vacc_m = empty_stats();
end

if has_applied_R
    stats.R_N = calc_positive_stats(R_applied(mask, 1));
    stats.R_E = calc_positive_stats(R_applied(mask, 2));
    stats.R_D = calc_positive_stats(R_applied(mask, 3));

    stats.sigma_R_N = calc_positive_stats(sigma_R_applied(mask, 1));
    stats.sigma_R_E = calc_positive_stats(sigma_R_applied(mask, 2));
    stats.sigma_R_D = calc_positive_stats(sigma_R_applied(mask, 3));
else
    stats.R_N = empty_stats();
    stats.R_E = empty_stats();
    stats.R_D = empty_stats();

    stats.sigma_R_N = empty_stats();
    stats.sigma_R_E = empty_stats();
    stats.sigma_R_D = empty_stats();
end

if has_innov && has_applied_R
    stats.norm_innov_N = calc_scalar_stats(norm_innov_N);
    stats.norm_innov_E = calc_scalar_stats(norm_innov_E);
    stats.norm_innov_D = calc_scalar_stats(norm_innov_D);
    stats.norm_innov_NE_rss = calc_positive_stats(norm_innov_NE_rss);
else
    stats.norm_innov_N = empty_stats();
    stats.norm_innov_E = empty_stats();
    stats.norm_innov_D = empty_stats();
    stats.norm_innov_NE_rss = empty_stats();
end

stats.vel_N = calc_scalar_stats(vel_N);
stats.vel_E = calc_scalar_stats(vel_E);
stats.vel_D = calc_scalar_stats(vel_D);
stats.horizontal_speed = calc_positive_stats(horizontal_speed);

stats.stationary.sample_count = sum(stationary_mask);
stats.stationary.speed_threshold_mps = stationary_speed_threshold_mps;
stats.stationary.vel_N = calc_scalar_stats(stationary_vel_N);
stats.stationary.vel_E = calc_scalar_stats(stationary_vel_E);
stats.stationary.vel_D = calc_scalar_stats(stationary_vel_D);
stats.stationary.horizontal_speed = calc_positive_stats(stationary_speed);

if has_correction_accepted
    accepted_eval = gnss_correction_accepted(mask);
    stats.gnss_correction_accepted_count = sum(accepted_eval);
    stats.gnss_correction_accepted_rate = mean(accepted_eval);
else
    stats.gnss_correction_accepted_count = NaN;
    stats.gnss_correction_accepted_rate = NaN;
end

if has_reject_count
    stats.gnss_reject_count_final = gnss_reject_count(find(isfinite(gnss_reject_count), 1, "last"));
else
    stats.gnss_reject_count_final = NaN;
end

%% ============================================================
%  13. Print Report
% =============================================================
fprintf("=================================================\n");
fprintf("Position Error: GNSS - EKF\n");
fprintf("=================================================\n");
fprintf("Axis        RMSE [m]     MAE [m]      Mean [m]     Std [m]      MaxAbs [m]\n");
fprintf("--------------------------------------------------------------------------------\n");
print_scalar_stat("N", stats.pos_N);
print_scalar_stat("E", stats.pos_E);
print_scalar_stat("D", stats.pos_D);
fprintf("--------------------------------------------------------------------------------\n");
fprintf("Horizontal RMSE = %.4f m, MAE = %.4f m, Mean = %.4f m, Max = %.4f m\n", ...
    stats.horizontal.rmse, stats.horizontal.mae, stats.horizontal.mean, stats.horizontal.max);
fprintf("=================================================\n\n");

if has_gnss_hacc || has_gnss_vacc || has_applied_R
    fprintf("=================================================\n");
    fprintf("GNSS Accuracy / Applied R Summary\n");
    fprintf("=================================================\n");

    if has_gnss_hacc
        fprintf("GNSS hAcc mean/median/max = %.4f / %.4f / %.4f m\n", ...
            mean_finite(gnss_hacc_m(mask)), median_finite(gnss_hacc_m(mask)), max_finite(gnss_hacc_m(mask)));
    end

    if has_gnss_vacc
        fprintf("GNSS vAcc mean/median/max = %.4f / %.4f / %.4f m\n", ...
            mean_finite(gnss_vacc_m(mask)), median_finite(gnss_vacc_m(mask)), max_finite(gnss_vacc_m(mask)));
    end

    if has_applied_R
        fprintf("Applied sigma_R_N mean/median/max = %.4f / %.4f / %.4f m\n", ...
            mean_finite(sigma_R_applied(mask, 1)), median_finite(sigma_R_applied(mask, 1)), max_finite(sigma_R_applied(mask, 1)));
        fprintf("Applied sigma_R_E mean/median/max = %.4f / %.4f / %.4f m\n", ...
            mean_finite(sigma_R_applied(mask, 2)), median_finite(sigma_R_applied(mask, 2)), max_finite(sigma_R_applied(mask, 2)));
        fprintf("Applied sigma_R_D mean/median/max = %.4f / %.4f / %.4f m\n", ...
            mean_finite(sigma_R_applied(mask, 3)), median_finite(sigma_R_applied(mask, 3)), max_finite(sigma_R_applied(mask, 3)));
    end

    if has_correction_accepted
        fprintf("GNSS correction accepted: %d / %d = %.2f %%\n", ...
            stats.gnss_correction_accepted_count, num_eval, 100.0 * stats.gnss_correction_accepted_rate);
    end

    if has_reject_count
        fprintf("Final GNSS innovation gate reject count = %.0f\n", stats.gnss_reject_count_final);
    end

    fprintf("=================================================\n\n");
end

if has_innov
    fprintf("=================================================\n");
    fprintf("GNSS Position Innovation\n");
    fprintf("=================================================\n");
    fprintf("Axis        RMSE [m]     MAE [m]      Mean [m]     Std [m]      MaxAbs [m]\n");
    fprintf("--------------------------------------------------------------------------------\n");
    print_scalar_stat("N", stats.innov_N);
    print_scalar_stat("E", stats.innov_E);
    print_scalar_stat("D", stats.innov_D);
    fprintf("--------------------------------------------------------------------------------\n");
    fprintf("Horizontal Innovation RMSE = %.4f m, MAE = %.4f m, Mean = %.4f m, Max = %.4f m\n", ...
        stats.innov_horizontal.rmse, stats.innov_horizontal.mae, ...
        stats.innov_horizontal.mean, stats.innov_horizontal.max);
    fprintf("=================================================\n\n");
end

if has_innov && has_applied_R
    fprintf("=================================================\n");
    fprintf("R-normalized Innovation Approximation\n");
    fprintf("=================================================\n");
    fprintf("Note: true NIS needs S = HPH' + R. Here only applied R is used.\n");
    fprintf("Axis        RMSE [-]     MAE [-]      Mean [-]     Std [-]      MaxAbs [-]\n");
    fprintf("--------------------------------------------------------------------------------\n");
    print_scalar_stat("N/R", stats.norm_innov_N);
    print_scalar_stat("E/R", stats.norm_innov_E);
    print_scalar_stat("D/R", stats.norm_innov_D);
    fprintf("--------------------------------------------------------------------------------\n");
    fprintf("NE RSS normalized mean = %.4f, max = %.4f\n", ...
        stats.norm_innov_NE_rss.mean, stats.norm_innov_NE_rss.max);
    fprintf("=================================================\n\n");
end

fprintf("=================================================\n");
fprintf("Velocity Summary\n");
fprintf("=================================================\n");
fprintf("Horizontal speed mean = %.4f m/s\n", stats.horizontal_speed.mean);
fprintf("Horizontal speed std  = %.4f m/s\n", stats.horizontal_speed.std);
fprintf("Horizontal speed max  = %.4f m/s\n", stats.horizontal_speed.max);
fprintf("V_N max abs           = %.4f m/s\n", stats.vel_N.max_abs);
fprintf("V_E max abs           = %.4f m/s\n", stats.vel_E.max_abs);
fprintf("V_D max abs           = %.4f m/s\n", stats.vel_D.max_abs);
fprintf("=================================================\n\n");

fprintf("=================================================\n");
fprintf("Stationary Velocity Summary\n");
fprintf("=================================================\n");
fprintf("Stationary threshold       = %.3f m/s\n", stationary_speed_threshold_mps);
fprintf("Stationary samples         = %d\n", stats.stationary.sample_count);
fprintf("Stationary speed RMSE      = %.4f m/s\n", stats.stationary.horizontal_speed.rmse);
fprintf("Stationary speed mean      = %.4f m/s\n", stats.stationary.horizontal_speed.mean);
fprintf("Stationary speed max       = %.4f m/s\n", stats.stationary.horizontal_speed.max);
fprintf("Stationary V_N max abs     = %.4f m/s\n", stats.stationary.vel_N.max_abs);
fprintf("Stationary V_E max abs     = %.4f m/s\n", stats.stationary.vel_E.max_abs);
fprintf("=================================================\n\n");

%% ============================================================
%  14. Figure 1: 2D Trajectory
% =============================================================
figure("Name", "01 2D Trajectory");
set(gcf, "Color", "w");
hold on; grid on; axis equal;

plot(ekf_pos(eval_mask, 2), ekf_pos(eval_mask, 1), "LineWidth", 1.5);
plot(gnss_pos(eval_mask & gnss_valid, 2), gnss_pos(eval_mask & gnss_valid, 1), ".", "MarkerSize", 8);

xlabel("East [m]");
ylabel("North [m]");
title("2D Trajectory: EKF vs GNSS");
legend("EKF", "GNSS", "Location", "best");

%% ============================================================
%  15. Figure 2: Position Error NE
% =============================================================
figure("Name", "02 Position Error NE");
set(gcf, "Color", "w");

subplot(3,1,1);
hold on; grid on;
plot(t(mask), err_N, "LineWidth", 1.2);
ylabel("N error [m]");
title("Position Error N = GNSS_N - EKF_N");

subplot(3,1,2);
hold on; grid on;
plot(t(mask), err_E, "LineWidth", 1.2);
ylabel("E error [m]");
title("Position Error E = GNSS_E - EKF_E");

subplot(3,1,3);
hold on; grid on;
plot(t(mask), horizontal_error, "LineWidth", 1.2);
ylabel("Horizontal [m]");
xlabel("Time [s]");
title("Horizontal Position Error");

%% ============================================================
%  16. Figure 3: Error Histogram
% =============================================================
figure("Name", "03 Error Histogram");
set(gcf, "Color", "w");

subplot(3,1,1);
histogram(err_N, 30);
grid on;
xlabel("N error [m]");
ylabel("Count");
title("N Position Error Histogram");

subplot(3,1,2);
histogram(err_E, 30);
grid on;
xlabel("E error [m]");
ylabel("Count");
title("E Position Error Histogram");

subplot(3,1,3);
histogram(horizontal_error, 30);
grid on;
xlabel("Horizontal error [m]");
ylabel("Count");
title("Horizontal Error Histogram");

%% ============================================================
%  17. Figure 4: GNSS Accuracy vs Error
% =============================================================
if plot_gnss_accuracy && has_gnss_hacc
    figure("Name", "04 GNSS hAcc vs Horizontal Error");
    set(gcf, "Color", "w");

    subplot(2,1,1);
    hold on; grid on;
    plot(t(mask), horizontal_error, "LineWidth", 1.2);
    plot(t(mask), gnss_hacc_m(mask), "LineWidth", 1.2);
    ylabel("[m]");
    title("Horizontal Error vs GNSS hAcc");
    legend("Horizontal error |GNSS - EKF|", "GNSS hAcc", "Location", "best");

    subplot(2,1,2);
    hold on; grid on;
    plot(t(mask), err_N, "LineWidth", 1.1);
    plot(t(mask), err_E, "LineWidth", 1.1);
    plot(t(mask), gnss_hacc_m(mask), "--", "LineWidth", 1.1);
    plot(t(mask), -gnss_hacc_m(mask), "--", "LineWidth", 1.1);
    ylabel("[m]");
    xlabel("Time [s]");
    title("N/E Error with ±hAcc Reference");
    legend("N error", "E error", "+hAcc", "-hAcc", "Location", "best");
end

%% ============================================================
%  18. Figure 5: Applied R vs GNSS Accuracy
% =============================================================
if plot_applied_R && has_applied_R
    figure("Name", "05 Applied GNSS R Sigma");
    set(gcf, "Color", "w");

    subplot(3,1,1);
    hold on; grid on;
    plot(t(mask), sigma_R_applied(mask, 1), "LineWidth", 1.2);
    if has_gnss_hacc
        plot(t(mask), gnss_hacc_m(mask), "--", "LineWidth", 1.1);
        legend("sqrt(R_N)", "hAcc", "Location", "best");
    else
        legend("sqrt(R_N)", "Location", "best");
    end
    ylabel("[m]");
    title("Applied GNSS Position Measurement Sigma - N");

    subplot(3,1,2);
    hold on; grid on;
    plot(t(mask), sigma_R_applied(mask, 2), "LineWidth", 1.2);
    if has_gnss_hacc
        plot(t(mask), gnss_hacc_m(mask), "--", "LineWidth", 1.1);
        legend("sqrt(R_E)", "hAcc", "Location", "best");
    else
        legend("sqrt(R_E)", "Location", "best");
    end
    ylabel("[m]");
    title("Applied GNSS Position Measurement Sigma - E");

    subplot(3,1,3);
    hold on; grid on;
    plot(t(mask), sigma_R_applied(mask, 3), "LineWidth", 1.2);
    if has_gnss_vacc
        plot(t(mask), gnss_vacc_m(mask), "--", "LineWidth", 1.1);
        legend("sqrt(R_D)", "vAcc", "Location", "best");
    else
        legend("sqrt(R_D)", "Location", "best");
    end
    ylabel("[m]");
    xlabel("Time [s]");
    title("Applied GNSS Position Measurement Sigma - D");
end

%% ============================================================
%  19. Figure 6: GNSS Innovation
% =============================================================
if has_innov
    figure("Name", "06 GNSS Innovation");
    set(gcf, "Color", "w");

    subplot(3,1,1);
    hold on; grid on;
    plot(t(innov_valid), innov_N, ".", "MarkerSize", 8);
    ylabel("Innov N [m]");
    title("GNSS Position Innovation N");

    subplot(3,1,2);
    hold on; grid on;
    plot(t(innov_valid), innov_E, ".", "MarkerSize", 8);
    ylabel("Innov E [m]");
    title("GNSS Position Innovation E");

    subplot(3,1,3);
    hold on; grid on;
    plot(t(innov_valid), innov_horizontal, ".", "MarkerSize", 8);
    ylabel("Innov NE [m]");
    xlabel("Time [s]");
    title("GNSS Horizontal Innovation");
end

%% ============================================================
%  20. Figure 7: Innovation vs hAcc / Applied Sigma
% =============================================================
if has_innov && (has_gnss_hacc || has_applied_R)
    figure("Name", "07 Innovation vs GNSS Accuracy");
    set(gcf, "Color", "w");

    subplot(2,1,1);
    hold on; grid on;
    plot(t(innov_valid), innov_horizontal, ".", "MarkerSize", 8);
    if has_gnss_hacc
        plot(t(innov_valid), gnss_hacc_m(innov_valid), "LineWidth", 1.2);
    end
    if has_applied_R
        sigma_h_R = sqrt(0.5 * (R_applied(:,1) + R_applied(:,2)));
        plot(t(innov_valid), sigma_h_R(innov_valid), "LineWidth", 1.2);
    end
    ylabel("[m]");
    title("Horizontal Innovation vs hAcc / Applied R Sigma");
    legend_items = ["Innovation NE"];
    if has_gnss_hacc
        legend_items(end+1) = "GNSS hAcc";
    end
    if has_applied_R
        legend_items(end+1) = "sqrt((R_N+R_E)/2)";
    end
    legend(legend_items, "Location", "best");

    subplot(2,1,2);
    hold on; grid on;
    if has_correction_accepted
        stairs(t(eval_mask), double(gnss_correction_accepted(eval_mask)), "LineWidth", 1.1);
        ylabel("Accepted");
        ylim([-0.1, 1.1]);
        title("GNSS Correction Accepted Flag");
    elseif has_reject_count
        stairs(t(eval_mask), gnss_reject_count(eval_mask), "LineWidth", 1.1);
        ylabel("Reject count");
        title("GNSS Innovation Gate Reject Count");
    else
        text(0.1, 0.5, "No accepted/reject flag column", "Units", "normalized");
        grid off;
    end
    xlabel("Time [s]");
end

%% ============================================================
%  21. Figure 8: R-normalized Innovation
% =============================================================
if plot_normalized_innovation && has_innov && has_applied_R
    figure("Name", "08 R-normalized Innovation Approx");
    set(gcf, "Color", "w");

    subplot(3,1,1);
    hold on; grid on;
    plot(t(norm_valid), norm_innov_N, ".", "MarkerSize", 8);
    yline(norm_gate_2, "--");
    yline(-norm_gate_2, "--");
    yline(norm_gate_3, ":");
    yline(-norm_gate_3, ":");
    ylabel("N / sqrt(R_N)");
    title("R-normalized Innovation N");

    subplot(3,1,2);
    hold on; grid on;
    plot(t(norm_valid), norm_innov_E, ".", "MarkerSize", 8);
    yline(norm_gate_2, "--");
    yline(-norm_gate_2, "--");
    yline(norm_gate_3, ":");
    yline(-norm_gate_3, ":");
    ylabel("E / sqrt(R_E)");
    title("R-normalized Innovation E");

    subplot(3,1,3);
    hold on; grid on;
    plot(t(norm_valid), norm_innov_NE_rss, ".", "MarkerSize", 8);
    yline(norm_gate_2, "--");
    yline(norm_gate_3, ":");
    ylabel("NE RSS [-]");
    xlabel("Time [s]");
    title("R-normalized Horizontal Innovation RSS");
end

%% ============================================================
%  22. Figure 9: Velocity NE
% =============================================================
figure("Name", "09 Velocity NE");
set(gcf, "Color", "w");

subplot(3,1,1);
hold on; grid on;
plot(t(vel_mask), vel_N, "LineWidth", 1.2);
ylabel("V_N [m/s]");
title("EKF Velocity N");

subplot(3,1,2);
hold on; grid on;
plot(t(vel_mask), vel_E, "LineWidth", 1.2);
ylabel("V_E [m/s]");
title("EKF Velocity E");

subplot(3,1,3);
hold on; grid on;
plot(t(vel_mask), horizontal_speed, "LineWidth", 1.2);
yline(stationary_speed_threshold_mps, "--");
ylabel("Speed NE [m/s]");
xlabel("Time [s]");
title("EKF Horizontal Speed");

%% ============================================================
%  23. Figure 10: Stationary Velocity Histogram
% =============================================================
if stats.stationary.sample_count > 5
    figure("Name", "10 Stationary Velocity Histogram");
    set(gcf, "Color", "w");

    subplot(3,1,1);
    histogram(stationary_vel_N, 30);
    grid on;
    xlabel("Stationary V_N [m/s]");
    ylabel("Count");
    title("Stationary V_N Histogram");

    subplot(3,1,2);
    histogram(stationary_vel_E, 30);
    grid on;
    xlabel("Stationary V_E [m/s]");
    ylabel("Count");
    title("Stationary V_E Histogram");

    subplot(3,1,3);
    histogram(stationary_speed, 30);
    grid on;
    xlabel("Stationary horizontal speed [m/s]");
    ylabel("Count");
    title("Stationary Horizontal Speed Histogram");
end

%% ============================================================
%  24. Optional Figure: Vertical Axis
% =============================================================
if plot_vertical_axis
    figure("Name", "11 Vertical Axis");
    set(gcf, "Color", "w");

    subplot(3,1,1);
    hold on; grid on;
    plot(t(mask), err_D, "LineWidth", 1.2);
    if has_gnss_vacc
        plot(t(mask), gnss_vacc_m(mask), "--", "LineWidth", 1.1);
        plot(t(mask), -gnss_vacc_m(mask), "--", "LineWidth", 1.1);
        legend("D error", "+vAcc", "-vAcc", "Location", "best");
    end
    ylabel("D error [m]");
    title("Position Error D = GNSS_D - EKF_D");

    subplot(3,1,2);
    hold on; grid on;
    plot(t(vel_mask), vel_D, "LineWidth", 1.2);
    ylabel("V_D [m/s]");
    title("EKF Velocity D");

    if has_innov
        subplot(3,1,3);
        hold on; grid on;
        plot(t(innov_valid), innov_D, ".", "MarkerSize", 8);
        ylabel("Innov D [m]");
        xlabel("Time [s]");
        title("GNSS Position Innovation D");
    end
end

%% ============================================================
%  25. Save Report
% =============================================================
report = struct();

report.csv_file = csv_file;
report.stats = stats;
report.eval_mask = mask;
report.time_s = t;

report.ekf_pos_ned = ekf_pos;
report.ekf_vel_ned = ekf_vel;
report.gnss_pos_ned = gnss_pos;

report.position_error_ned = pos_error;
report.horizontal_error = horizontal_error;

report.has_gnss_hacc = has_gnss_hacc;
report.has_gnss_vacc = has_gnss_vacc;
report.gnss_hacc_m = gnss_hacc_m;
report.gnss_vacc_m = gnss_vacc_m;

report.has_applied_R = has_applied_R;
report.R_applied_gnss_pos_ned = R_applied;
report.sigma_R_applied_gnss_pos_ned = sigma_R_applied;

report.has_innovation = has_innov;
report.innovation_pos_ned = innov_pos;

report.has_normalized_innovation = has_innov && has_applied_R;
report.normalized_innovation_valid = norm_valid;
report.normalized_innovation_N = norm_innov_N;
report.normalized_innovation_E = norm_innov_E;
report.normalized_innovation_D = norm_innov_D;
report.normalized_innovation_NE_rss = norm_innov_NE_rss;

report.has_correction_accepted = has_correction_accepted;
report.gnss_correction_accepted = gnss_correction_accepted;

report.has_reject_count = has_reject_count;
report.gnss_reject_count = gnss_reject_count;

report.stationary_mask = stationary_mask;
report.stationary_speed_threshold_mps = stationary_speed_threshold_mps;

% save("ekf_csv_performance_report_with_gnss_cov.mat", "report", "stats");

% fprintf("Saved performance report: ekf_csv_performance_report_with_gnss_cov.mat\n");

%% ========================================================================
%  Local Functions
% ========================================================================

function require_cols(cols, names)

names = string(names);

for k = 1:numel(names)
    if ~ismember(names(k), cols)
        error("CSV에 필요한 컬럼이 없습니다: %s", names(k));
    end
end

end

function pos_ned = lla_to_local_ned(lat, lon, h, lat0, lon0, h0)

a = 6378137.0;
f = 1.0 / 298.257223563;
e2 = f * (2.0 - f);

sin_lat0 = sin(lat0);

N0 = a / sqrt(1.0 - e2 * sin_lat0^2);
M0 = a * (1.0 - e2) / (1.0 - e2 * sin_lat0^2)^(3.0 / 2.0);

d_lat = lat - lat0;
d_lon = lon - lon0;
d_h   = h - h0;

north = d_lat * (M0 + h0);
east  = d_lon * (N0 + h0) * cos(lat0);
down  = -d_h;

pos_ned = [north; east; down];

end

function s = calc_scalar_stats(x)

x = x(:);
x = x(isfinite(x));

if isempty(x)
    s = empty_stats();
    return;
end

s.rmse = sqrt(mean(x.^2));
s.mae = mean(abs(x));
s.mean = mean(x);
s.std = std(x);
s.max_abs = max(abs(x));
s.max = max(x);
s.min = min(x);

end

function s = calc_positive_stats(x)

x = x(:);
x = x(isfinite(x));

if isempty(x)
    s = empty_stats();
    return;
end

s.rmse = sqrt(mean(x.^2));
s.mae = mean(abs(x));
s.mean = mean(x);
s.std = std(x);
s.max_abs = max(abs(x));
s.max = max(x);
s.min = min(x);

end

function s = empty_stats()

s.rmse = NaN;
s.mae = NaN;
s.mean = NaN;
s.std = NaN;
s.max_abs = NaN;
s.max = NaN;
s.min = NaN;

end

function print_scalar_stat(name, s)

fprintf("%-7s %10.4f  %10.4f  %10.4f  %10.4f  %10.4f\n", ...
    name, s.rmse, s.mae, s.mean, s.std, s.max_abs);

end

function y = mean_finite(x)

x = x(:);
x = x(isfinite(x));

if isempty(x)
    y = NaN;
else
    y = mean(x);
end

end

function y = median_finite(x)

x = x(:);
x = x(isfinite(x));

if isempty(x)
    y = NaN;
else
    y = median(x);
end

end

function y = max_finite(x)

x = x(:);
x = x(isfinite(x));

if isempty(x)
    y = NaN;
else
    y = max(x);
end

end