clear; clc; close all;

%% STRIX FC GNSS Quality Check
% Checks GNSS alone: fix type, satellites, hAcc/vAcc/sAcc, GNSS velocity.
% No output files are saved.

this_dir = fileparts(mfilename("fullpath"));
addpath(fullfile(this_dir, "common"));

%% User settings
csv_file = "";              % leave empty to open file picker
cfg = struct();
cfg.analysis_start_sec = 0.0;
cfg.analysis_end_sec = inf;
cfg.use_ekf_ready = false;
cfg.use_gnss_ref_ready = false;
cfg.use_gnss_valid = false;
cfg.title_prefix = "02 GNSS Quality Check";

fix_type_good = 3;
num_sv_min = 6;
num_sv_good = 10;
hacc_good = 1.0;             % m
hacc_warn = 2.0;             % m
sacc_good = 0.20;            % m/s
sacc_warn = 0.50;            % m/s

%% Load
log = load_vtol_log(csv_file, cfg);
T = log.T; cols = log.cols; N = log.N; t = log.t; mask = log.mask;

fix_type = strix_col(T, cols, "gnss_fix_type", nan(N, 1));
num_sv = strix_col(T, cols, "gnss_num_sv", nan(N, 1));
hacc = strix_col(T, cols, "gnss_hacc_m", nan(N, 1));
vacc = strix_col(T, cols, "gnss_vacc_m", nan(N, 1));
sacc = strix_col(T, cols, "gnss_sacc_mps", nan(N, 1));
gnss_valid = strix_boolcol(T, cols, "gnss_valid", false(N, 1));
gnss_ref_ready = strix_boolcol(T, cols, "gnss_ref_ready", false(N, 1));
gnss_update = strix_boolcol(T, cols, "gnss_update_executed", false(N, 1));
gnss_velocity_used = strix_boolcol(T, cols, "gnss_velocity_used", false(N, 1));

[gnss_speed_h, speed_source] = strix_gnss_speed_h(T, cols, N);
gnss_vel_d = strix_col(T, cols, "gnss_vel_d_mps", strix_col(T, cols, "gnss_vel_d", nan(N, 1)));

quality_mask = mask & isfinite(fix_type);

fprintf("GNSS velocity source: %s\n\n", speed_source);

fprintf("=================================================\n");
fprintf("GNSS Quality Summary\n");
fprintf("=================================================\n");
fprintf("fix_type >= %d ratio              : %.2f %%\n", fix_type_good, 100 * mean(fix_type(quality_mask) >= fix_type_good, "omitnan"));
fprintf("num_sv >= %d ratio                : %.2f %%\n", num_sv_min, 100 * mean(num_sv(mask) >= num_sv_min, "omitnan"));
fprintf("num_sv >= %d ratio               : %.2f %%\n", num_sv_good, 100 * mean(num_sv(mask) >= num_sv_good, "omitnan"));
strix_print_metric("gnss_num_sv", num_sv(mask), "sat");
strix_print_metric("gnss_hacc_m", hacc(mask), "m");
strix_print_metric("gnss_vacc_m", vacc(mask), "m");
strix_print_metric("gnss_sacc_mps", sacc(mask), "m/s");
strix_print_metric("gnss_speed_h", gnss_speed_h(mask), "m/s");
strix_print_metric("gnss_vel_d", gnss_vel_d(mask), "m/s");
fprintf("gnss_valid samples                : %d\n", sum(mask & gnss_valid));
fprintf("gnss_ref_ready samples            : %d\n", sum(mask & gnss_ref_ready));
fprintf("gnss_update_executed samples      : %d\n", sum(mask & gnss_update));
fprintf("gnss_velocity_used samples        : %d\n", sum(mask & gnss_velocity_used));

hacc_p95 = strix_pctf(hacc(mask), 95);
sacc_p95 = strix_pctf(sacc(mask), 95);
fix_ratio = mean(fix_type(quality_mask) >= fix_type_good, "omitnan");
sv_ratio = mean(num_sv(mask) >= num_sv_min, "omitnan");

gnss_pass = fix_ratio > 0.95 && sv_ratio > 0.95 && hacc_p95 < hacc_good && sacc_p95 < sacc_good;
gnss_warn = fix_ratio > 0.80 && sv_ratio > 0.80 && hacc_p95 < hacc_warn && sacc_p95 < sacc_warn;

fprintf("-------------------------------------------------\n");
fprintf("GNSS status                       : %s\n", strix_status(gnss_pass, gnss_warn));
fprintf("=================================================\n\n");

%% Plots
figure("Name", "02 GNSS Quality - Time Series", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, fix_type, "LineWidth", 1.0); hold on;
yline(fix_type_good, "--", "3D fix");
grid on; ylabel("fix type");
title("GNSS fix type");

nexttile;
plot(t, num_sv, "LineWidth", 1.0); hold on;
yline(num_sv_min, "--", "min");
yline(num_sv_good, "--", "good");
grid on; ylabel("satellites");
title("Number of satellites");

nexttile;
plot(t, hacc, "LineWidth", 1.0); hold on;
plot(t, vacc, "LineWidth", 1.0);
yline(hacc_good, "--", "hAcc good");
yline(hacc_warn, "--r", "hAcc warn");
grid on; ylabel("accuracy [m]");
legend("hAcc", "vAcc", "Location", "best");
title("GNSS position accuracy");

nexttile;
plot(t, sacc, "LineWidth", 1.0); hold on;
yline(sacc_good, "--", "good");
yline(sacc_warn, "--r", "warn");
yyaxis right;
plot(t, gnss_speed_h, "LineWidth", 1.0);
ylabel("speed H [m/s]");
yyaxis left;
grid on; xlabel("time [s]"); ylabel("sAcc [m/s]");
title("GNSS speed accuracy and horizontal speed");

figure("Name", "02 GNSS Quality - Scatter", "Color", "w");
tiledlayout(1, 3, "TileSpacing", "compact");

nexttile;
scatter(num_sv(mask), hacc(mask), 12, "filled");
grid on; xlabel("num SV"); ylabel("hAcc [m]");
title("Satellites vs hAcc");

nexttile;
scatter(hacc(mask), sacc(mask), 12, "filled");
grid on; xlabel("hAcc [m]"); ylabel("sAcc [m/s]");
title("hAcc vs sAcc");

nexttile;
scatter(sacc(mask), gnss_speed_h(mask), 12, "filled");
grid on; xlabel("sAcc [m/s]"); ylabel("GNSS speed H [m/s]");
title("sAcc vs GNSS speed");
