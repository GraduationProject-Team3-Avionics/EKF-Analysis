clear; clc; close all;

%% ============================================================
%  GNSS / INS EKF XY Consistency Visualization
%  - Ground Truth 없이 GNSS hAcc 기반 EKF 수평(XY) 신뢰도 평가
%  - D축은 barometer로 별도 처리 예정이므로 분석 제외
% ============================================================

filename = "data/TEST_10.CSV";

use_only_gnss_update = true;

%% ============================================================
%  1. CSV Load
% ============================================================
T = readtable(filename, "VariableNamingRule", "preserve");

t = T.timestamp_ms / 1000;
t = t - t(1);

vars = string(T.Properties.VariableNames);

%% ============================================================
%  2. Valid Mask
% ============================================================
mask = true(height(T), 1);

if ismember("gnss_valid", vars)
    mask = mask & (T.gnss_valid == 1);
end

if ismember("gnss_ref_ready", vars)
    mask = mask & (T.gnss_ref_ready == 1);
end

if ismember("ekf_ready", vars)
    mask = mask & (T.ekf_ready == 1);
end

if use_only_gnss_update && ismember("gnss_update_executed", vars)
    mask = mask & (T.gnss_update_executed == 1);
end

%% ============================================================
%  3. XY Data Extract
% ============================================================
gnss_n = T.ekf_gnss_pos_n;
gnss_e = T.ekf_gnss_pos_e;

ekf_n = T.ekf_pos_n;
ekf_e = T.ekf_pos_e;

% GNSS - EKF
err_n = gnss_n - ekf_n;
err_e = gnss_e - ekf_e;

% Horizontal XY difference
h_err = sqrt(err_n.^2 + err_e.^2);

% GNSS receiver-reported horizontal accuracy
hacc = T.gnss_hacc_m;

% Speed accuracy는 참고용으로만 표시
if ismember("gnss_sacc_mps", vars)
    sacc = T.gnss_sacc_mps;
else
    sacc = nan(height(T), 1);
end

%% ============================================================
%  4. XY Innovation Extract
% ============================================================
innov_n = T.ekf_innov_pos_n;
innov_e = T.ekf_innov_pos_e;

sigma_n = T.ekf_sigma_applied_gnss_pos_n;
sigma_e = T.ekf_sigma_applied_gnss_pos_e;

norm_innov_n = innov_n ./ sigma_n;
norm_innov_e = innov_e ./ sigma_e;

% 2D normalized innovation magnitude
norm_innov_xy = sqrt(norm_innov_n.^2 + norm_innov_e.^2);

%% ============================================================
%  5. Statistics
% ============================================================
h_err_valid = h_err(mask);
hacc_valid = hacc(mask);

ratio_hacc = h_err ./ hacc;
ratio_hacc_valid = ratio_hacc(mask);

coverage_1hacc = mean(h_err_valid <= hacc_valid, "omitnan") * 100;
coverage_2hacc = mean(h_err_valid <= 2*hacc_valid, "omitnan") * 100;
coverage_3hacc = mean(h_err_valid <= 3*hacc_valid, "omitnan") * 100;

h_rmse = rms(h_err_valid);
h_mae  = mean(abs(h_err_valid), "omitnan");
h_max  = max(abs(h_err_valid));

n_rmse = rms(err_n(mask));
e_rmse = rms(err_e(mask));

n_mean = mean(err_n(mask), "omitnan");
e_mean = mean(err_e(mask), "omitnan");

n_std = std(err_n(mask), "omitnan");
e_std = std(err_e(mask), "omitnan");

n_norm_3sigma = mean(abs(norm_innov_n(mask)) <= 3, "omitnan") * 100;
e_norm_3sigma = mean(abs(norm_innov_e(mask)) <= 3, "omitnan") * 100;

%% ============================================================
%  6. Console Summary
% ============================================================
fprintf("\n=================================================\n");
fprintf("GNSS / INS EKF XY Consistency Summary\n");
fprintf("=================================================\n");
fprintf("File              : %s\n", filename);
fprintf("Total rows         : %d\n", height(T));
fprintf("Valid samples      : %d\n", sum(mask));
fprintf("Time range         : %.3f ~ %.3f sec\n", min(t(mask)), max(t(mask)));
fprintf("-------------------------------------------------\n");

fprintf("[GNSS Horizontal Accuracy]\n");
fprintf("GNSS hAcc median   : %.3f m\n", median(hacc(mask), "omitnan"));
fprintf("GNSS hAcc mean     : %.3f m\n", mean(hacc(mask), "omitnan"));
fprintf("GNSS hAcc max      : %.3f m\n", max(hacc(mask)));

if any(~isnan(sacc(mask)))
    fprintf("GNSS sAcc median   : %.3f m/s\n", median(sacc(mask), "omitnan"));
end

fprintf("-------------------------------------------------\n");
fprintf("[XY Position Difference: GNSS - EKF]\n");
fprintf("Horizontal RMSE    : %.3f m\n", h_rmse);
fprintf("Horizontal MAE     : %.3f m\n", h_mae);
fprintf("Horizontal Max     : %.3f m\n", h_max);
fprintf("N RMSE             : %.3f m\n", n_rmse);
fprintf("E RMSE             : %.3f m\n", e_rmse);
fprintf("N Mean             : %.3f m\n", n_mean);
fprintf("E Mean             : %.3f m\n", e_mean);
fprintf("N Std              : %.3f m\n", n_std);
fprintf("E Std              : %.3f m\n", e_std);

fprintf("-------------------------------------------------\n");
fprintf("[Horizontal Error / GNSS hAcc]\n");
fprintf("Median Error/hAcc  : %.3f\n", median(ratio_hacc_valid, "omitnan"));
fprintf("Mean Error/hAcc    : %.3f\n", mean(ratio_hacc_valid, "omitnan"));
fprintf("Max Error/hAcc     : %.3f\n", max(ratio_hacc_valid));

fprintf("-------------------------------------------------\n");
fprintf("[Coverage]\n");
fprintf("|GNSS - EKF| < 1*hAcc : %.2f %%\n", coverage_1hacc);
fprintf("|GNSS - EKF| < 2*hAcc : %.2f %%\n", coverage_2hacc);
fprintf("|GNSS - EKF| < 3*hAcc : %.2f %%\n", coverage_3hacc);

fprintf("-------------------------------------------------\n");
fprintf("[Normalized Innovation]\n");
fprintf("|N innovation| < 3 sigma : %.2f %%\n", n_norm_3sigma);
fprintf("|E innovation| < 3 sigma : %.2f %%\n", e_norm_3sigma);
fprintf("=================================================\n\n");

%% ============================================================
%  7. Figure 1
%  XY Consistency Overview
% ============================================================
figure("Name", "XY EKF Consistency Overview", "Color", "w");
tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

% 1) GNSS hAcc
nexttile;
plot(t(mask), hacc(mask), "LineWidth", 1.6); hold on;

if any(~isnan(sacc(mask)))
    plot(t(mask), sacc(mask), "LineWidth", 1.2);
    legend("GNSS hAcc [m]", "GNSS sAcc [m/s]", "Location", "best");
else
    legend("GNSS hAcc [m]", "Location", "best");
end

grid on;
xlabel("Time [s]");
ylabel("Accuracy");
title("GNSS Horizontal Accuracy Estimate");

% 2) Horizontal error vs hAcc
nexttile;
plot(t(mask), h_err(mask), "LineWidth", 1.8); hold on;
plot(t(mask), hacc(mask), "--", "LineWidth", 1.5);
plot(t(mask), 2*hacc(mask), ":", "LineWidth", 1.5);

grid on;
xlabel("Time [s]");
ylabel("Horizontal error [m]");
title("Horizontal Difference: GNSS Position - EKF Position");
legend("|GNSS - EKF| horizontal", "GNSS hAcc", "2 × GNSS hAcc", "Location", "best");

% 3) Error / hAcc
nexttile;
plot(t(mask), ratio_hacc(mask), "LineWidth", 1.7); hold on;
yline(1, "--", "1 × hAcc");
yline(2, ":", "2 × hAcc");
yline(3, "-.", "3 × hAcc");

grid on;
xlabel("Time [s]");
ylabel("Error / hAcc");
title("Horizontal Error Compared to GNSS hAcc");
legend("|GNSS - EKF| / hAcc", "1 × hAcc", "2 × hAcc", "3 × hAcc", "Location", "best");

%% ============================================================
%  8. Figure 2
%  N/E Position Difference
% ============================================================
figure("Name", "XY Position Difference by Axis", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t(mask), err_n(mask), "LineWidth", 1.5); hold on;
yline(0, "k--");
grid on;
xlabel("Time [s]");
ylabel("N error [m]");
title("N-axis Difference: GNSS - EKF");

nexttile;
plot(t(mask), err_e(mask), "LineWidth", 1.5); hold on;
yline(0, "k--");
grid on;
xlabel("Time [s]");
ylabel("E error [m]");
title("E-axis Difference: GNSS - EKF");

%% ============================================================
%  9. Figure 3
%  N/E Normalized Innovation
% ============================================================
figure("Name", "XY Normalized Innovation", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot_normalized_innovation(t(mask), norm_innov_n(mask), "N-axis Normalized Innovation");

nexttile;
plot_normalized_innovation(t(mask), norm_innov_e(mask), "E-axis Normalized Innovation");

%% ============================================================
%  10. Figure 4
%  XY Summary Bar Chart
% ============================================================
figure("Name", "XY Summary Metrics", "Color", "w");
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

% hAcc coverage
nexttile;
coverage_values = [coverage_1hacc, coverage_2hacc, coverage_3hacc];

bar(coverage_values);
grid on;
ylim([0 105]);
xticklabels(["< 1 hAcc", "< 2 hAcc", "< 3 hAcc"]);
ylabel("Coverage [%]");
title("Horizontal Error Coverage");

for i = 1:numel(coverage_values)
    text(i, coverage_values(i) + 2, sprintf("%.1f%%", coverage_values(i)), ...
        "HorizontalAlignment", "center", "FontWeight", "bold");
end

% normalized innovation coverage
nexttile;
norm_values = [n_norm_3sigma, e_norm_3sigma];

bar(norm_values);
grid on;
ylim([0 105]);
xticklabels(["N", "E"]);
ylabel("Samples inside ±3σ [%]");
title("N/E Normalized Innovation Coverage");

for i = 1:numel(norm_values)
    text(i, norm_values(i) + 2, sprintf("%.1f%%", norm_values(i)), ...
        "HorizontalAlignment", "center", "FontWeight", "bold");
end

%% ============================================================
%  11. Final Interpretation
% ============================================================
fprintf("\n=================================================\n");
fprintf("XY Interpretation Guide\n");
fprintf("=================================================\n");

if coverage_1hacc > 90
    fprintf("[GOOD] XY EKF-GNSS difference is mostly within GNSS hAcc.\n");
elseif coverage_2hacc > 95
    fprintf("[OK] XY EKF-GNSS difference is mostly within 2*hAcc.\n");
else
    fprintf("[CHECK] XY EKF-GNSS difference often exceeds GNSS hAcc range.\n");
end

if min([n_norm_3sigma, e_norm_3sigma]) > 95
    fprintf("[GOOD] N/E normalized innovations are mostly inside ±3 sigma.\n");
else
    fprintf("[CHECK] N/E normalized innovations exceed ±3 sigma frequently.\n");
end

fprintf("\nImportant limitation:\n");
fprintf("This does NOT prove true ground-truth accuracy.\n");
fprintf("It only shows horizontal EKF-GNSS statistical consistency using GNSS hAcc.\n");
fprintf("D-axis is intentionally excluded because vertical estimation will be evaluated with barometer later.\n");
fprintf("=================================================\n");

%% ============================================================
%  Helper Function
% ============================================================
function plot_normalized_innovation(t, z, title_text)

    plot(t, z, "LineWidth", 1.5); hold on;

    yline(0, "k-", "LineWidth", 0.8);
    yline(1, "--");
    yline(-1, "--");
    yline(2, ":");
    yline(-2, ":");
    yline(3, "-.");
    yline(-3, "-.");

    grid on;
    xlabel("Time [s]");
    ylabel("Innovation / sigma");
    title(title_text);

    ylim_auto = max(4, ceil(max(abs(z), [], "omitnan")));
    ylim([-ylim_auto, ylim_auto]);

    legend("Normalized innovation", "0", "+1σ", "-1σ", "+2σ", "-2σ", "+3σ", "-3σ", ...
        "Location", "best");
end