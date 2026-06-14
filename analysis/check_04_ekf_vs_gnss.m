clear; clc; close all;

%% STRIX FC EKF vs GNSS Check
% Compares EKF and GNSS: velocity, position reference, innovation, correction flags.
% No output files are saved.

this_dir = fileparts(mfilename("fullpath"));
addpath(fullfile(this_dir, "common"));

%% User settings
csv_file = "";              % leave empty to open file picker
cfg = struct();
cfg.analysis_start_sec = 0.0;
cfg.analysis_end_sec = inf;
cfg.use_ekf_ready = true;
cfg.use_gnss_ref_ready = true;
cfg.use_gnss_valid = false;
cfg.title_prefix = "04 EKF vs GNSS Check";

dv_h_good = 0.20;            % m/s
dv_h_warn = 0.50;            % m/s
innov_vel_h_warn = 0.50;     % m/s
innov_pos_h_warn = 2.00;     % m

%% Load
log = load_vtol_log(csv_file, cfg);
T = log.T; cols = log.cols; N = log.N; t = log.t; mask = log.mask;

ekf_vel_n = strix_col(T, cols, "ekf_vel_n", nan(N, 1));
ekf_vel_e = strix_col(T, cols, "ekf_vel_e", nan(N, 1));
ekf_vel_d = strix_col(T, cols, "ekf_vel_d", nan(N, 1));

gnss_vel_n = strix_col(T, cols, "gnss_vel_n_mps", strix_col(T, cols, "gnss_vel_n", nan(N, 1)));
gnss_vel_e = strix_col(T, cols, "gnss_vel_e_mps", strix_col(T, cols, "gnss_vel_e", nan(N, 1)));
gnss_vel_d = strix_col(T, cols, "gnss_vel_d_mps", strix_col(T, cols, "gnss_vel_d", nan(N, 1)));

dv_n = ekf_vel_n - gnss_vel_n;
dv_e = ekf_vel_e - gnss_vel_e;
dv_d = ekf_vel_d - gnss_vel_d;
dv_h = hypot(dv_n, dv_e);

ekf_pos_n = strix_col(T, cols, "ekf_pos_n", nan(N, 1));
ekf_pos_e = strix_col(T, cols, "ekf_pos_e", nan(N, 1));
ekf_pos_d = strix_col(T, cols, "ekf_pos_d", nan(N, 1));

gnss_pos_n = strix_col(T, cols, "ekf_gnss_pos_n", strix_col(T, cols, "gnss_pos_n", nan(N, 1)));
gnss_pos_e = strix_col(T, cols, "ekf_gnss_pos_e", strix_col(T, cols, "gnss_pos_e", nan(N, 1)));
gnss_pos_d = strix_col(T, cols, "ekf_gnss_pos_d", strix_col(T, cols, "gnss_pos_d", nan(N, 1)));

dp_n = ekf_pos_n - gnss_pos_n;
dp_e = ekf_pos_e - gnss_pos_e;
dp_d = ekf_pos_d - gnss_pos_d;
dp_h = hypot(dp_n, dp_e);

innov_pos_n = strix_col(T, cols, "ekf_innov_pos_n", nan(N, 1));
innov_pos_e = strix_col(T, cols, "ekf_innov_pos_e", nan(N, 1));
innov_pos_d = strix_col(T, cols, "ekf_innov_pos_d", nan(N, 1));
innov_vel_n = strix_col(T, cols, "ekf_innov_vel_n", nan(N, 1));
innov_vel_e = strix_col(T, cols, "ekf_innov_vel_e", nan(N, 1));
innov_vel_d = strix_col(T, cols, "ekf_innov_vel_d", nan(N, 1));

innov_pos_h = hypot(innov_pos_n, innov_pos_e);
innov_vel_h = hypot(innov_vel_n, innov_vel_e);

delta_vel_n = strix_col(T, cols, "delta_vel_gnss_update_n", nan(N, 1));
delta_vel_e = strix_col(T, cols, "delta_vel_gnss_update_e", nan(N, 1));
delta_vel_h = hypot(delta_vel_n, delta_vel_e);

gnss_update = strix_boolcol(T, cols, "gnss_update_executed", false(N, 1));
gnss_accepted = strix_boolcol(T, cols, "gnss_correction_accepted", false(N, 1));
gnss_velocity_used = strix_boolcol(T, cols, "gnss_velocity_used", false(N, 1));

update_mask = mask & gnss_update;

fprintf("=================================================\n");
fprintf("EKF vs GNSS Summary\n");
fprintf("=================================================\n");
fprintf("GNSS update executed samples       : %d\n", sum(update_mask));
fprintf("GNSS correction accepted samples   : %d\n", sum(mask & gnss_accepted));
if sum(update_mask) > 0
    fprintf("GNSS accepted / executed ratio     : %.2f %%\n", 100 * sum(mask & gnss_accepted) / sum(update_mask));
else
    fprintf("GNSS accepted / executed ratio     : NaN %%\n");
end
fprintf("GNSS velocity used samples         : %d\n", sum(mask & gnss_velocity_used));
strix_print_metric("EKF-GNSS velocity diff H", dv_h(mask), "m/s");
strix_print_metric("EKF-GNSS velocity diff D", dv_d(mask), "m/s");
strix_print_metric("EKF-GNSS position diff H", dp_h(mask), "m");
strix_print_metric("EKF-GNSS position diff D", dp_d(mask), "m");
strix_print_metric("innovation position H", innov_pos_h(update_mask), "m");
strix_print_metric("innovation velocity H", innov_vel_h(update_mask), "m/s");
strix_print_metric("delta vel GNSS update H", delta_vel_h(update_mask), "m/s");

[worst_dv, worst_idx_local] = max(dv_h(mask), [], "omitnan");
idx_all = find(mask);
if ~isempty(idx_all) && isfinite(worst_dv)
    worst_idx = idx_all(worst_idx_local);
    fprintf("Worst velocity mismatch time       : %.3f sec, dv_h=%.4f m/s\n", t(worst_idx), worst_dv);
end

dv_p95 = strix_pctf(dv_h(mask), 95);
innov_vel_p95 = strix_pctf(innov_vel_h(update_mask), 95);
innov_pos_p95 = strix_pctf(innov_pos_h(update_mask), 95);

compare_pass = dv_p95 < dv_h_good;
compare_warn = dv_p95 < dv_h_warn && innov_vel_p95 < innov_vel_h_warn && innov_pos_p95 < innov_pos_h_warn;

fprintf("-------------------------------------------------\n");
fprintf("EKF vs GNSS status                : %s\n", strix_status(compare_pass, compare_warn));
fprintf("=================================================\n\n");

%% Plots
figure("Name", "04 EKF vs GNSS - Velocity", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, ekf_vel_n, "LineWidth", 1.0); hold on;
plot(t, gnss_vel_n, "--", "LineWidth", 1.0);
grid on; ylabel("N [m/s]");
legend("EKF", "GNSS", "Location", "best");
title("Velocity N");

nexttile;
plot(t, ekf_vel_e, "LineWidth", 1.0); hold on;
plot(t, gnss_vel_e, "--", "LineWidth", 1.0);
grid on; ylabel("E [m/s]");
legend("EKF", "GNSS", "Location", "best");
title("Velocity E");

nexttile;
plot(t, ekf_vel_d, "LineWidth", 1.0); hold on;
plot(t, gnss_vel_d, "--", "LineWidth", 1.0);
grid on; ylabel("D [m/s]");
legend("EKF", "GNSS", "Location", "best");
title("Velocity D");

nexttile;
plot(t, dv_h, "LineWidth", 1.0); hold on;
yline(dv_h_good, "--", "good");
yline(dv_h_warn, "--r", "warn");
grid on; xlabel("time [s]"); ylabel("diff H [m/s]");
title("EKF-GNSS horizontal velocity difference");

figure("Name", "04 EKF vs GNSS - Position and Innovation", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, dp_h, "LineWidth", 1.0); hold on;
plot(t, dp_d, "LineWidth", 1.0);
grid on; ylabel("pos diff");
legend("H", "D", "Location", "best");
title("EKF position - GNSS reference position");

nexttile;
plot(t, innov_pos_h, "LineWidth", 1.0); hold on;
plot(t, innov_pos_d, "LineWidth", 1.0);
grid on; ylabel("pos innov");
legend("H", "D", "Location", "best");
title("GNSS position innovation");

nexttile;
plot(t, innov_vel_h, "LineWidth", 1.0); hold on;
plot(t, innov_vel_d, "LineWidth", 1.0);
grid on; ylabel("vel innov");
legend("H", "D", "Location", "best");
title("GNSS velocity innovation");

nexttile;
stairs(t, double(gnss_update), "LineWidth", 1.0); hold on;
stairs(t, double(gnss_accepted), "LineWidth", 1.0);
stairs(t, double(gnss_velocity_used), "LineWidth", 1.0);
grid on; xlabel("time [s]"); ylabel("flag");
legend("update", "accepted", "velocity used", "Location", "best");
title("GNSS correction flags");

figure("Name", "04 EKF vs GNSS - Scatter", "Color", "w");
tiledlayout(1, 2, "TileSpacing", "compact");

nexttile;
scatter(gnss_vel_n(mask), ekf_vel_n(mask), 12, "filled"); hold on;
lims = [min([gnss_vel_n(mask); ekf_vel_n(mask)], [], "omitnan"), max([gnss_vel_n(mask); ekf_vel_n(mask)], [], "omitnan")];
plot(lims, lims, "k--");
grid on; xlabel("GNSS vel N [m/s]"); ylabel("EKF vel N [m/s]");
title("Velocity N agreement");

nexttile;
scatter(gnss_vel_e(mask), ekf_vel_e(mask), 12, "filled"); hold on;
lims = [min([gnss_vel_e(mask); ekf_vel_e(mask)], [], "omitnan"), max([gnss_vel_e(mask); ekf_vel_e(mask)], [], "omitnan")];
plot(lims, lims, "k--");
grid on; xlabel("GNSS vel E [m/s]"); ylabel("EKF vel E [m/s]");
title("Velocity E agreement");
