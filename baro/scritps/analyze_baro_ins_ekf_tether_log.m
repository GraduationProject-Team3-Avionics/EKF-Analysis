%% STRIX FC barometer preprocessing + baro-INS EKF tether log check
% Use with logs that include the barometer preprocessing, baro EKF aid source,
% and velocity-control fields.
%
% Test condition assumed here:
%   - Vehicle is hanging from a bar / tether.
%   - True vertical motion should be small.
%   - Large baro/EKF height drift is suspicious unless the frame was moved.

clear; clc; close all;

%% User settings
csv_file = "";
if strlength(csv_file) == 0
    csv_file = string(getenv("STRIX_BARO_CSV"));
end
if strlength(csv_file) == 0
    csv_file = "data/baro_test_01.CSV";
end
if ~isfile(csv_file)
    fallback = fullfile(pwd, ".cache", "baro_test_01.CSV");
    if isfile(fallback)
        csv_file = string(fallback);
    else
        [f, p] = uigetfile("*.CSV;*.csv", "Select STRIX FC baro log");
        if isequal(f, 0)
            error("No CSV file selected.");
        end
        csv_file = string(fullfile(p, f));
    end
end

analysis_start_sec = 0.0;
analysis_end_sec = inf;

% Tether/static sanity thresholds. Tune after a few logs.
baro_rel_p95_limit_m = 0.50;
baro_rel_drift_limit_m = 0.40;
ekf_z_p95_limit_m = 0.80;
reject_ratio_limit = 0.20;
test_ratio_p95_limit = 1.0;
vel_xy_p95_limit_mps = 0.35;

save_figures = false;
fig_dir = "baro_ins_ekf_figures";

%% Load
opts = detectImportOptions(csv_file);
opts.VariableNamingRule = "preserve";
T = readtable(csv_file, opts);

req = ["timestamp_ms", ...
    "baro_temp_c", "baro_pressure_pa", "baro_alt_raw_m", "baro_alt_bias_m", ...
    "baro_rel_alt_m", "baro_rel_alt_lpf_m", "baro_age_ms", "baro_updated", ...
    "baro_bias_calibrated", "baro_ready", ...
    "ekf_ready", "ekf_pos_d", "ekf_vel_d", ...
    "baro_innov_m", "baro_R_base", "baro_R_applied", "baro_test_ratio", ...
    "baro_update_used", "baro_rejected", "baro_height_pred_m", "baro_meas_m"];
assert_has_vars(T, req);

t = col(T, "timestamp_ms") * 1e-3;
t = t - t(1);
in_window = t >= analysis_start_sec & t <= analysis_end_sec;
T = T(in_window, :);
t = t(in_window);

dt = diff(t);
fs_med = 1 / median(dt(dt > 0), "omitnan");

%% Extract columns
baro_temp_c = col(T, "baro_temp_c");
baro_pressure_pa = col(T, "baro_pressure_pa");
baro_alt_raw_m = col(T, "baro_alt_raw_m");
baro_alt_bias_m = col(T, "baro_alt_bias_m");
baro_rel_alt_m = col(T, "baro_rel_alt_m");
baro_rel_alt_lpf_m = col(T, "baro_rel_alt_lpf_m");
baro_age_ms = col(T, "baro_age_ms");
baro_updated = flag(T, "baro_updated");
baro_bias_calibrated = flag(T, "baro_bias_calibrated");
baro_ready = flag(T, "baro_ready");

ekf_ready = flag(T, "ekf_ready");
ekf_pos_d = col(T, "ekf_pos_d");
ekf_vel_d = col(T, "ekf_vel_d");
ekf_height_m = -ekf_pos_d;       % NED D positive down -> height positive up

baro_innov_m = col(T, "baro_innov_m");
baro_R_base = col(T, "baro_R_base");
baro_R_applied = col(T, "baro_R_applied");
baro_test_ratio = col(T, "baro_test_ratio");
baro_update_used = flag(T, "baro_update_used");
baro_rejected = flag(T, "baro_rejected");
baro_height_pred_m = col(T, "baro_height_pred_m");
baro_meas_m = col(T, "baro_meas_m");

motor_on = optional_flag(T, "motor_on_detected");
vel_ctrl_active = optional_flag(T, "vel_ctrl_active");
vel_ctrl_valid = optional_flag(T, "vel_ctrl_valid");
ekf_pwm_mean = optional_col(T, "ekf_pwm_mean");
vel_filt_n = optional_col(T, "vel_filt_n_mps");
vel_filt_e = optional_col(T, "vel_filt_e_mps");
vel_xy = hypot(vel_filt_n, vel_filt_e);

%% Masks
m_all = true(height(T), 1);
m_baro_ready = baro_ready & baro_bias_calibrated;
m_ekf = ekf_ready & m_baro_ready;
m_motor = m_ekf & motor_on;
m_vel = m_ekf & vel_ctrl_active;
m_used = m_ekf & baro_update_used;
m_rej = m_ekf & baro_rejected;

%% Console report
fprintf("\n=== STRIX Baro-INS EKF Tether Log Check ===\n");
fprintf("File: %s\n", csv_file);
fprintf("Rows: %d, duration: %.2f s, median rate: %.2f Hz\n", height(T), t(end)-t(1), fs_med);

fprintf("\n--- Basic data validity ---\n");
fprintf("baro_ready ratio          : %6.2f %%\n", 100 * mean(baro_ready));
fprintf("baro_bias_calibrated ratio: %6.2f %%\n", 100 * mean(baro_bias_calibrated));
fprintf("ekf_ready ratio           : %6.2f %%\n", 100 * mean(ekf_ready));
fprintf("baro_updated ratio        : %6.2f %%\n", 100 * mean(baro_updated));
fprintf("baro_age_ms max / p95     : %.1f / %.1f ms\n", max(baro_age_ms, [], "omitnan"), pct(baro_age_ms, 95));

pressure_med = median(baro_pressure_pa, "omitnan");
pressure_span = max(baro_pressure_pa, [], "omitnan") - min(baro_pressure_pa, [], "omitnan");
fprintf("pressure median / span    : %.3f / %.6f Pa\n", pressure_med, pressure_span);
fprintf("temperature mean / span   : %.3f / %.3f degC\n", mean(baro_temp_c, "omitnan"), range_omitnan(baro_temp_c));

if pressure_med < 30000 || pressure_med > 125000
    fprintf("WARNING: baro_pressure_pa is outside BMP390L physical pressure range.\n");
    fprintf("         Check unit/scaling/log field. BMP390L should be roughly 30000..125000 Pa.\n");
end

if pressure_span < 1e-6
    fprintf("WARNING: baro_pressure_pa is constant in this log. Pressure logging may be wrong\n");
    fprintf("         even if altitude fields are changing.\n");
end

fprintf("\n--- Tether/static height metrics ---\n");
print_height_metrics("All", t, m_all, baro_rel_alt_lpf_m, ekf_height_m);
print_height_metrics("EKF ready", t, m_ekf, baro_rel_alt_lpf_m, ekf_height_m);
print_height_metrics("Motor on", t, m_motor, baro_rel_alt_lpf_m, ekf_height_m);
print_height_metrics("Vel ctrl", t, m_vel, baro_rel_alt_lpf_m, ekf_height_m);

fprintf("\n--- Baro fusion metrics while EKF ready ---\n");
print_fusion_metrics(m_ekf, baro_update_used, baro_rejected, baro_innov_m, ...
    baro_test_ratio, baro_R_base, baro_R_applied);

fprintf("\n--- Velocity-control context ---\n");
if any(isfinite(vel_xy))
    fprintf("vel_xy RMS / P95 / max during vel_ctrl: %.3f / %.3f / %.3f m/s\n", ...
        rms_omitnan(vel_xy(m_vel)), pct(vel_xy(m_vel), 95), max(vel_xy(m_vel), [], "omitnan"));
else
    fprintf("Velocity fields not available.\n");
end

fprintf("\n--- Simple pass/fail hints ---\n");
baro_p95 = pct(abs_zeroed(baro_rel_alt_lpf_m(m_ekf)), 95);
baro_drift = endpoint_drift(t(m_ekf), baro_rel_alt_lpf_m(m_ekf));
ekf_p95 = pct(abs_zeroed(ekf_height_m(m_ekf)), 95);
reject_ratio = mean(baro_rejected(m_ekf));
test_ratio_p95 = pct(baro_test_ratio(m_ekf), 95);
vel_xy_p95 = pct(vel_xy(m_vel), 95);

check_line("baro_rel_alt_lpf P95", baro_p95, baro_rel_p95_limit_m, "m");
check_line("baro_rel_alt_lpf drift", abs(baro_drift), baro_rel_drift_limit_m, "m");
check_line("EKF height P95", ekf_p95, ekf_z_p95_limit_m, "m");
check_line("baro_rejected ratio", reject_ratio, reject_ratio_limit, "");
check_line("baro_test_ratio P95", test_ratio_p95, test_ratio_p95_limit, "");
if any(m_vel)
    check_line("vel_xy P95 during vel_ctrl", vel_xy_p95, vel_xy_p95_limit_mps, "m/s");
end

%% Plots
if save_figures && ~isfolder(fig_dir)
    mkdir(fig_dir);
end

fig1 = figure("Name", "Barometer preprocessing overview", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, baro_alt_raw_m, "Color", [0.6 0.6 0.6]); hold on;
plot(t, baro_alt_bias_m, "k--");
plot(t, baro_rel_alt_m, "b");
plot(t, baro_rel_alt_lpf_m, "r", "LineWidth", 1.1);
grid on; ylabel("m");
legend("alt raw", "bias", "rel", "rel lpf", "Location", "best");
title("BMP390L baro preprocessing");

nexttile;
plot(t, baro_pressure_pa, "b"); grid on;
ylabel("Pa");
title("Logged pressure. If this is not ~30000..125000 Pa, check unit/scaling/logging.");

nexttile;
plot(t, baro_temp_c, "r"); grid on;
ylabel("degC");
title("Barometer temperature");

nexttile;
stairs(t, baro_ready, "g"); hold on;
stairs(t, baro_bias_calibrated, "b");
stairs(t, baro_updated, "k");
plot(t, baro_age_ms / max(max(baro_age_ms), 1), "m");
grid on; ylim([-0.1 1.2]); xlabel("time [s]");
legend("ready", "bias calibrated", "updated", "age normalized", "Location", "best");

fig2 = figure("Name", "Baro-INS EKF fusion check", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, baro_meas_m, "r", "LineWidth", 1.1); hold on;
plot(t, baro_height_pred_m, "b");
plot(t, ekf_height_m, "k--");
grid on; ylabel("m");
legend("baro meas", "baro height pred", "EKF height=-D", "Location", "best");
title("Baro measurement vs EKF prediction");

nexttile;
plot(t, baro_innov_m, "b"); grid on;
ylabel("m");
title("Baro innovation");

nexttile;
plot(t, baro_test_ratio, "k"); hold on;
yline(1.0, "r--", "gate");
grid on; ylabel("ratio");
title("Baro innovation test ratio");

nexttile;
stairs(t, baro_update_used, "g"); hold on;
stairs(t, baro_rejected, "r");
stairs(t, ekf_ready, "k");
grid on; ylim([-0.1 1.2]); xlabel("time [s]");
legend("update used", "rejected", "ekf ready", "Location", "best");

fig3 = figure("Name", "Motor/velocity-control interaction", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, ekf_pwm_mean, "k"); hold on;
stairs(t, motor_on * max(ekf_pwm_mean, [], "omitnan"), "r");
grid on; ylabel("PWM");
legend("mean PWM", "motor on scaled", "Location", "best");
title("Motor state");

nexttile;
plot(t, baro_rel_alt_lpf_m, "r"); hold on;
plot(t, ekf_height_m, "b");
grid on; ylabel("m");
legend("baro rel lpf", "EKF height", "Location", "best");
title("Height while motors/velocity control run");

nexttile;
plot(t, vel_xy, "k"); hold on;
plot(t, vel_filt_n, "b");
plot(t, vel_filt_e, "r");
grid on; ylabel("m/s");
legend("vel xy", "vel N", "vel E", "Location", "best");

nexttile;
stairs(t, vel_ctrl_active, "g"); hold on;
stairs(t, vel_ctrl_valid, "b");
plot(t, baro_test_ratio, "k");
yline(1.0, "r--");
grid on; xlabel("time [s]");
legend("vel ctrl active", "vel ctrl valid", "baro test ratio", "gate", "Location", "best");

fig4 = figure("Name", "Scatter/correlation checks", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");

nexttile;
scatter(ekf_pwm_mean(m_ekf), baro_innov_m(m_ekf), 8, t(m_ekf), "filled");
grid on; xlabel("mean PWM"); ylabel("baro innov [m]");
title("PWM vs baro innovation"); colorbar;

nexttile;
scatter(ekf_pwm_mean(m_ekf), baro_test_ratio(m_ekf), 8, t(m_ekf), "filled");
grid on; xlabel("mean PWM"); ylabel("test ratio");
title("PWM vs baro test ratio"); colorbar;

nexttile;
scatter(baro_rel_alt_lpf_m(m_ekf), ekf_height_m(m_ekf), 8, t(m_ekf), "filled");
grid on; xlabel("baro rel lpf [m]"); ylabel("EKF height [m]");
title("Baro vs EKF height"); colorbar;

nexttile;
scatter(baro_temp_c(m_ekf), baro_rel_alt_lpf_m(m_ekf), 8, t(m_ekf), "filled");
grid on; xlabel("baro temp [degC]"); ylabel("baro rel lpf [m]");
title("Temperature vs baro relative height"); colorbar;

if save_figures
    saveas(fig1, fullfile(fig_dir, "01_baro_preprocessing.png"));
    saveas(fig2, fullfile(fig_dir, "02_baro_ins_ekf_fusion.png"));
    saveas(fig3, fullfile(fig_dir, "03_motor_velocity_interaction.png"));
    saveas(fig4, fullfile(fig_dir, "04_scatter_checks.png"));
end

%% Local functions
function assert_has_vars(T, names)
missing = names(~ismember(names, string(T.Properties.VariableNames)));
if ~isempty(missing)
    error("Missing required columns:\n%s", strjoin(missing, newline));
end
end

function x = col(T, name)
x = T.(name);
if ~isnumeric(x)
    x = str2double(string(x));
end
x = double(x);
end

function x = optional_col(T, name)
if ismember(name, string(T.Properties.VariableNames))
    x = col(T, name);
else
    x = nan(height(T), 1);
end
end

function y = flag(T, name)
y = col(T, name) ~= 0;
end

function y = optional_flag(T, name)
if ismember(name, string(T.Properties.VariableNames))
    y = flag(T, name);
else
    y = false(height(T), 1);
end
end

function p = pct(x, q)
x = x(isfinite(x));
if isempty(x)
    p = nan;
else
    p = prctile(x, q);
end
end

function r = rms_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    r = nan;
else
    r = sqrt(mean(x.^2));
end
end

function y = range_omitnan(x)
x = x(isfinite(x));
if isempty(x)
    y = nan;
else
    y = max(x) - min(x);
end
end

function xz = abs_zeroed(x)
x = x(isfinite(x));
if isempty(x)
    xz = nan;
else
    xz = abs(x - median(x, "omitnan"));
end
end

function d = endpoint_drift(t, x)
good = isfinite(t) & isfinite(x);
t = t(good);
x = x(good);
if numel(x) < 5
    d = nan;
    return;
end
n = max(3, round(0.05 * numel(x)));
d = mean(x(end-n+1:end), "omitnan") - mean(x(1:n), "omitnan");
end

function print_height_metrics(label, t, mask, baro_h, ekf_h)
if ~any(mask)
    fprintf("%-10s: no samples\n", label);
    return;
end
bh = baro_h(mask);
eh = ekf_h(mask);
tm = t(mask);
fprintf("%-10s: duration %.1fs | baro RMS/P95/span/drift %.3f/%.3f/%.3f/%+.3f m | EKF RMS/P95/span/drift %.3f/%.3f/%.3f/%+.3f m\n", ...
    label, tm(end)-tm(1), ...
    rms_omitnan(bh - median(bh, "omitnan")), pct(abs_zeroed(bh), 95), range_omitnan(bh), endpoint_drift(tm, bh), ...
    rms_omitnan(eh - median(eh, "omitnan")), pct(abs_zeroed(eh), 95), range_omitnan(eh), endpoint_drift(tm, eh));
end

function print_fusion_metrics(mask, used, rejected, innov, test_ratio, R_base, R_applied)
if ~any(mask)
    fprintf("No EKF-ready samples.\n");
    return;
end
fprintf("update used ratio       : %6.2f %%\n", 100 * mean(used(mask)));
fprintf("rejected ratio          : %6.2f %%\n", 100 * mean(rejected(mask)));
fprintf("innov RMS / P95 / max   : %.3f / %.3f / %.3f m\n", ...
    rms_omitnan(innov(mask)), pct(abs(innov(mask)), 95), max(abs(innov(mask)), [], "omitnan"));
fprintf("test ratio mean/P95/max : %.3f / %.3f / %.3f\n", ...
    mean(test_ratio(mask), "omitnan"), pct(test_ratio(mask), 95), max(test_ratio(mask), [], "omitnan"));
fprintf("R base mean             : %.3f\n", mean(R_base(mask), "omitnan"));
fprintf("R applied mean/P95/max  : %.3f / %.3f / %.3f\n", ...
    mean(R_applied(mask), "omitnan"), pct(R_applied(mask), 95), max(R_applied(mask), [], "omitnan"));
end

function check_line(name, value, limit, unit)
if isnan(value)
    verdict = "N/A";
elseif value <= limit
    verdict = "OK";
else
    verdict = "CHECK";
end
if strlength(unit) > 0
    fprintf("%-28s: %8.3f <= %-8.3f %s  [%s]\n", name, value, limit, unit, verdict);
else
    fprintf("%-28s: %8.3f <= %-8.3f     [%s]\n", name, value, limit, verdict);
end
end

