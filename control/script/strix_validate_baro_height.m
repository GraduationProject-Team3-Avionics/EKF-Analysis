clear; clc; close all;

%% STRIX baro-INS height validation script
% Purpose:
%   Validate vertical height estimation using barometer + INS related fields.
%   GNSS altitude is intentionally ignored.
%
% Main checks:
%   - Barometer pressure/log scaling
%   - Barometer update freshness and rejection
%   - Baro relative altitude stability
%   - EKF vertical position/velocity stability
%   - Baro innovation magnitude and gain behavior
%
% No toolbox dependency:
%   This script avoids prctile(), corr(), rms(), etc.

%% User settings
csv_file = "";    % Example: "data/260620_2.CSV"

analysis_start_sec = 0.0;
analysis_end_sec = inf;

% Main judging mask. Usually use EKF-ready samples only.
use_ekf_ready_only = true;
use_baro_ready_only = true;

show_plots = true;
save_plot = false;
plot_file = "";   % empty -> auto name beside CSV

% Thresholds. Adjust after collecting several good tether/hover logs.
limit.min_duration_sec = 10.0;
limit.pressure_min_pa = 30000.0;
limit.pressure_max_pa = 120000.0;
limit.pressure_span_warn_pa = 80.0;       % static/tether should not drift too much.
limit.temperature_span_warn_c = 5.0;

limit.baro_ready_min_pct = 99.0;
limit.baro_update_used_min_pct = 90.0;    % among judged EKF samples, if column exists.
limit.baro_reject_max_pct = 1.0;
limit.baro_age_p95_warn_ms = 80.0;
limit.baro_age_max_warn_ms = 150.0;

limit.baro_rel_lpf_std_warn_m = 0.35;
limit.baro_rel_lpf_p95abs_warn_m = 1.0;
limit.ekf_height_std_warn_m = 0.50;
limit.ekf_height_p95abs_warn_m = 1.5;
limit.ekf_vel_d_p95abs_warn_mps = 0.50;
limit.innov_p95abs_warn_m = 0.75;
limit.innov_maxabs_warn_m = 2.0;

limit.height_baro_ekf_diff_p95_warn_m = 0.75;

%% Load CSV
if strlength(string(csv_file)) == 0
    env_file = string(getenv("STRIX_LOG_CSV"));
    if strlength(env_file) > 0
        csv_file = env_file;
    end
end

if strlength(string(csv_file)) == 0 || ~isfile(csv_file)
    [f, p] = uigetfile({"*.CSV;*.csv", "CSV logs (*.CSV, *.csv)"}, ...
        "Select STRIX log CSV");
    if isequal(f, 0)
        error("No CSV file selected.");
    end
    csv_file = string(fullfile(p, f));
end

T = readtable(csv_file, "VariableNamingRule", "preserve");
required = [
    "timestamp_ms"
    "baro_temp_c"
    "baro_pressure_pa"
    "baro_rel_alt_m"
    "baro_age_ms"
    "baro_updated"
    "baro_bias_calibrated"
    "baro_ready"
    "ekf_ready"
    "ekf_pos_d"
    "ekf_vel_d"
    "baro_innov_m"
    "baro_R_base"
    "baro_R_applied"
    "baro_update_used"
    "baro_rejected"
    "baro_height_pred_m"
    "baro_meas_m"
];
assert_required_columns(T, required);

t = (T.timestamp_ms - T.timestamp_ms(1)) * 1e-3;
time_mask = t >= analysis_start_sec & t <= analysis_end_sec;

judge_mask = time_mask;
if use_ekf_ready_only
    judge_mask = judge_mask & logical(T.ekf_ready);
end
if use_baro_ready_only
    judge_mask = judge_mask & logical(T.baro_ready);
end

if nnz(judge_mask) < 2
    error("Not enough samples in selected analysis mask.");
end

%% Derived height conventions
% NED D is positive downward. Height-up estimate is therefore -ekf_pos_d.
ekf_height_m = -T.ekf_pos_d;
ekf_v_up_mps = -T.ekf_vel_d;

if has_column(T, "baro_rel_alt_lpf_m")
    baro_rel_lpf_m = T.baro_rel_alt_lpf_m;
else
    baro_rel_lpf_m = T.baro_rel_alt_m;
end

if has_column(T, "ekf_prop_height_m")
    prop_height_m = T.ekf_prop_height_m;
else
    prop_height_m = nan(height(T), 1);
end

if has_column(T, "ekf_prop_vel_d")
    prop_v_up_mps = -T.ekf_prop_vel_d;
else
    prop_v_up_mps = nan(height(T), 1);
end

height_baro_ekf_diff = ekf_height_m - baro_rel_lpf_m;
meas_pred_diff = T.baro_meas_m - T.baro_height_pred_m;

%% Stats
S.file = string(csv_file);
S.rows = height(T);
S.duration = t(end);
S.median_hz = 1 / finite_median(diff(t));
S.mask_samples = nnz(judge_mask);
S.mask_duration = mask_duration(t, judge_mask);

S.baro_ready_pct = 100 * finite_mean(double(logical(T.baro_ready(time_mask))));
S.baro_bias_cal_pct = 100 * finite_mean(double(logical(T.baro_bias_calibrated(time_mask))));
S.ekf_ready_pct = 100 * finite_mean(double(logical(T.ekf_ready(time_mask))));
S.baro_updated_pct = 100 * finite_mean(double(logical(T.baro_updated(time_mask))));
S.baro_update_used_pct = 100 * finite_mean(double(logical(T.baro_update_used(judge_mask))));
S.baro_reject_pct = 100 * finite_mean(double(logical(T.baro_rejected(judge_mask))));

S.pressure = signed_stats(T.baro_pressure_pa, judge_mask);
S.temperature = signed_stats(T.baro_temp_c, judge_mask);
S.baro_age = mag_stats(T.baro_age_ms, judge_mask);
S.baro_rel = signed_stats(T.baro_rel_alt_m, judge_mask);
S.baro_rel_lpf = signed_stats(baro_rel_lpf_m, judge_mask);
S.ekf_height = signed_stats(ekf_height_m, judge_mask);
S.ekf_v_up = signed_stats(ekf_v_up_mps, judge_mask);
S.height_diff = signed_stats(height_baro_ekf_diff, judge_mask);
S.innov = signed_stats(T.baro_innov_m, judge_mask);
S.meas_pred_diff = signed_stats(meas_pred_diff, judge_mask);
S.R_base = mag_stats(T.baro_R_base, judge_mask);
S.R_applied = mag_stats(T.baro_R_applied, judge_mask);

if has_column(T, "baro_K_h")
    S.K_h = mag_stats(T.baro_K_h, judge_mask);
else
    S.K_h = empty_stats();
end
if has_column(T, "baro_K_v")
    S.K_v = mag_stats(T.baro_K_v, judge_mask);
else
    S.K_v = empty_stats();
end
if has_column(T, "ekf_p_cov_d")
    S.P_d = mag_stats(T.ekf_p_cov_d, judge_mask);
else
    S.P_d = empty_stats();
end
if has_column(T, "ekf_v_cov_d")
    S.V_d = mag_stats(T.ekf_v_cov_d, judge_mask);
else
    S.V_d = empty_stats();
end
if any(isfinite(prop_height_m(judge_mask)))
    S.prop_height = signed_stats(prop_height_m, judge_mask);
    S.prop_v_up = signed_stats(prop_v_up_mps, judge_mask);
else
    S.prop_height = empty_stats();
    S.prop_v_up = empty_stats();
end

%% Validation checks
results = {};
results{end+1} = check_result("judge duration", ...
    S.mask_duration >= limit.min_duration_sec, ...
    sprintf("%.2f s >= %.2f s", S.mask_duration, limit.min_duration_sec), ...
    "Not enough judged data.");

results{end+1} = check_result("baro ready", ...
    S.baro_ready_pct >= limit.baro_ready_min_pct, ...
    sprintf("%.2f%% >= %.2f%%", S.baro_ready_pct, limit.baro_ready_min_pct), ...
    "Barometer is not consistently ready.");

results{end+1} = check_warn("baro bias calibrated", ...
    S.baro_bias_cal_pct >= limit.baro_ready_min_pct, ...
    sprintf("%.2f%% >= %.2f%%", S.baro_bias_cal_pct, limit.baro_ready_min_pct), ...
    "Barometer bias calibration is not consistently done.");

results{end+1} = check_result("pressure range", ...
    S.pressure.min >= limit.pressure_min_pa && S.pressure.max <= limit.pressure_max_pa, ...
    sprintf("pressure min/max %.1f / %.1f Pa", S.pressure.min, S.pressure.max), ...
    "Barometer pressure scale is suspicious.");

results{end+1} = check_warn("pressure span", ...
    S.pressure.span <= limit.pressure_span_warn_pa, ...
    sprintf("pressure span %.2f <= %.2f Pa", S.pressure.span, limit.pressure_span_warn_pa), ...
    "Pressure drift/span is large for a static/tether test.");

results{end+1} = check_warn("temperature span", ...
    S.temperature.span <= limit.temperature_span_warn_c, ...
    sprintf("temperature span %.2f <= %.2f degC", S.temperature.span, limit.temperature_span_warn_c), ...
    "Temperature span is large.");

results{end+1} = check_warn("baro update used", ...
    S.baro_update_used_pct >= limit.baro_update_used_min_pct, ...
    sprintf("%.2f%% >= %.2f%%", S.baro_update_used_pct, limit.baro_update_used_min_pct), ...
    "EKF is not using baro updates often enough in judged mask.");

results{end+1} = check_warn("baro rejected", ...
    S.baro_reject_pct <= limit.baro_reject_max_pct, ...
    sprintf("%.2f%% <= %.2f%%", S.baro_reject_pct, limit.baro_reject_max_pct), ...
    "Barometer updates are being rejected.");

results{end+1} = check_warn("baro age P95", ...
    S.baro_age.p95 <= limit.baro_age_p95_warn_ms, ...
    sprintf("baro_age P95 %.1f <= %.1f ms", S.baro_age.p95, limit.baro_age_p95_warn_ms), ...
    "Baro sample age is high.");

results{end+1} = check_warn("baro age max", ...
    S.baro_age.max <= limit.baro_age_max_warn_ms, ...
    sprintf("baro_age max %.1f <= %.1f ms", S.baro_age.max, limit.baro_age_max_warn_ms), ...
    "Baro sample age has spikes.");

results{end+1} = check_warn("baro relative stability", ...
    S.baro_rel_lpf.std <= limit.baro_rel_lpf_std_warn_m && S.baro_rel_lpf.p95_abs <= limit.baro_rel_lpf_p95abs_warn_m, ...
    sprintf("baro_rel_lpf std/P95abs %.3f / %.3f m", S.baro_rel_lpf.std, S.baro_rel_lpf.p95_abs), ...
    "Baro relative altitude is noisy or drifting.");

results{end+1} = check_warn("EKF height stability", ...
    S.ekf_height.std <= limit.ekf_height_std_warn_m && S.ekf_height.p95_abs <= limit.ekf_height_p95abs_warn_m, ...
    sprintf("EKF height std/P95abs %.3f / %.3f m", S.ekf_height.std, S.ekf_height.p95_abs), ...
    "EKF height is noisy or drifting.");

results{end+1} = check_warn("EKF vertical velocity", ...
    S.ekf_v_up.p95_abs <= limit.ekf_vel_d_p95abs_warn_mps, ...
    sprintf("EKF vertical velocity P95abs %.3f <= %.3f m/s", S.ekf_v_up.p95_abs, limit.ekf_vel_d_p95abs_warn_mps), ...
    "EKF vertical velocity is large for a static/tether test.");

results{end+1} = check_warn("baro innovation", ...
    S.innov.p95_abs <= limit.innov_p95abs_warn_m && S.innov.max_abs <= limit.innov_maxabs_warn_m, ...
    sprintf("innovation P95abs/maxabs %.3f / %.3f m", S.innov.p95_abs, S.innov.max_abs), ...
    "Baro innovation is large.");

results{end+1} = check_warn("baro-EKF height difference", ...
    S.height_diff.p95_abs <= limit.height_baro_ekf_diff_p95_warn_m, ...
    sprintf("height diff P95abs %.3f <= %.3f m", S.height_diff.p95_abs, limit.height_baro_ekf_diff_p95_warn_m), ...
    "EKF height and baro relative altitude differ noticeably.");

%% Print report
fprintf("=== STRIX Baro-INS Height Validation ===\n");
fprintf("File        : %s\n", S.file);
fprintf("Rows        : %d\n", S.rows);
fprintf("Duration    : %.2f s\n", S.duration);
fprintf("Median rate : %.2f Hz\n", S.median_hz);
fprintf("Judge mask  : %d samples, %.2f s duration\n", S.mask_samples, S.mask_duration);
fprintf("Note        : GNSS altitude/vertical accuracy are intentionally ignored.\n\n");

print_results(results);

fprintf("\n--- Readiness / update ---\n");
fprintf("baro_ready              : %6.2f %%\n", S.baro_ready_pct);
fprintf("baro_bias_calibrated    : %6.2f %%\n", S.baro_bias_cal_pct);
fprintf("ekf_ready               : %6.2f %%\n", S.ekf_ready_pct);
fprintf("baro_updated            : %6.2f %%\n", S.baro_updated_pct);
fprintf("baro_update_used, judged: %6.2f %%\n", S.baro_update_used_pct);
fprintf("baro_rejected, judged   : %6.2f %%\n", S.baro_reject_pct);

fprintf("\n--- Barometer raw/stability ---\n");
print_signed("pressure", S.pressure, "Pa");
print_signed("temperature", S.temperature, "degC");
print_mag("baro_age", S.baro_age, "ms");
print_signed("baro_rel_alt", S.baro_rel, "m");
print_signed("baro_rel_lpf", S.baro_rel_lpf, "m");

fprintf("\n--- EKF vertical states, height-up convention ---\n");
print_signed("EKF height = -D", S.ekf_height, "m");
print_signed("EKF v_up = -vD", S.ekf_v_up, "m/s");
print_signed("EKF height - baro", S.height_diff, "m");
if isfinite(S.prop_height.mean)
    print_signed("prop height", S.prop_height, "m");
    print_signed("prop v_up", S.prop_v_up, "m/s");
end

fprintf("\n--- Baro correction ---\n");
print_signed("baro innov", S.innov, "m");
print_signed("meas - pred", S.meas_pred_diff, "m");
print_mag("baro R base", S.R_base, "");
print_mag("baro R applied", S.R_applied, "");
if isfinite(S.K_h.mean), print_mag("baro K_h", S.K_h, ""); end
if isfinite(S.K_v.mean), print_mag("baro K_v", S.K_v, ""); end
if isfinite(S.P_d.mean), print_mag("P_d", S.P_d, ""); end
if isfinite(S.V_d.mean), print_mag("V_d", S.V_d, ""); end

fprintf("\n--- Active segments in judge mask ---\n");
segments = logical_segments(judge_mask);
for i = 1:size(segments, 1)
    s = segments(i, 1);
    e = segments(i, 2);
    m = false(height(T), 1);
    m(s:e) = true;
    fprintf("seg %d: %.2f ~ %.2f s, dur %.2f s\n", i, t(s), t(e), t(e) - t(s));
    fprintf("  baro_rel_lpf mean/std/P95abs = %.3f / %.3f / %.3f m\n", ...
        finite_mean(baro_rel_lpf_m(m)), finite_std(baro_rel_lpf_m(m)), finite_percentile(abs(baro_rel_lpf_m(m)), 95));
    fprintf("  EKF height mean/std/P95abs   = %.3f / %.3f / %.3f m\n", ...
        finite_mean(ekf_height_m(m)), finite_std(ekf_height_m(m)), finite_percentile(abs(ekf_height_m(m)), 95));
    fprintf("  innov mean/P95abs/maxabs     = %.3f / %.3f / %.3f m\n", ...
        finite_mean(T.baro_innov_m(m)), finite_percentile(abs(T.baro_innov_m(m)), 95), finite_max(abs(T.baro_innov_m(m))));
end

%% Plot
if show_plots
    fig = figure("Name", "STRIX Baro-INS Height Validation", "Color", "w");
    try
        fig.Position = [80 60 1450 950];
    catch
    end

    ax1 = subplot(5, 1, 1); hold(ax1, "on");
    plot(ax1, t, T.baro_pressure_pa, "Color", [0.15 0.15 0.15], "LineWidth", 1.0);
    add_mask_background(ax1, t, judge_mask);
    ylabel(ax1, "Pressure [Pa]");
    title(ax1, "Barometer pressure");
    grid(ax1, "on");

    ax2 = subplot(5, 1, 2); hold(ax2, "on");
    plot(ax2, t, T.baro_rel_alt_m, "Color", [0.85 0.33 0.10], "LineWidth", 0.8);
    plot(ax2, t, baro_rel_lpf_m, "b", "LineWidth", 1.2);
    plot(ax2, t, ekf_height_m, "Color", [0.49 0.18 0.56], "LineWidth", 1.1);
    add_mask_background(ax2, t, judge_mask);
    ylabel(ax2, "Height [m]");
    title(ax2, "Baro relative altitude and EKF height-up estimate");
    legend(ax2, "baro rel raw", "baro rel lpf", "EKF height=-D", "Location", "eastoutside");
    grid(ax2, "on");

    ax3 = subplot(5, 1, 3); hold(ax3, "on");
    plot(ax3, t, T.ekf_vel_d, "Color", [0.85 0.33 0.10], "LineWidth", 0.8);
    plot(ax3, t, ekf_v_up_mps, "b", "LineWidth", 1.1);
    if any(isfinite(prop_v_up_mps))
        plot(ax3, t, prop_v_up_mps, "Color", [0.47 0.67 0.19], "LineWidth", 0.8);
        legend(ax3, "EKF vD", "EKF v_up=-vD", "prop v_up", "Location", "eastoutside");
    else
        legend(ax3, "EKF vD", "EKF v_up=-vD", "Location", "eastoutside");
    end
    add_mask_background(ax3, t, judge_mask);
    ylabel(ax3, "Vel [m/s]");
    title(ax3, "Vertical velocity");
    grid(ax3, "on");

    ax4 = subplot(5, 1, 4); hold(ax4, "on");
    plot(ax4, t, T.baro_innov_m, "r", "LineWidth", 1.0);
    plot(ax4, t, meas_pred_diff, "Color", [0.49 0.18 0.56], "LineWidth", 0.8);
    add_mask_background(ax4, t, judge_mask);
    ylabel(ax4, "Innovation [m]");
    title(ax4, "Baro innovation and measurement-prediction difference");
    legend(ax4, "baro innov", "baro meas - pred", "Location", "eastoutside");
    grid(ax4, "on");

    ax5 = subplot(5, 1, 5); hold(ax5, "on");
    yyaxis(ax5, "left");
    plot(ax5, t, T.baro_age_ms, "Color", [0.25 0.25 0.25], "LineWidth", 0.9);
    ylabel(ax5, "Age [ms]");
    yyaxis(ax5, "right");
    plot(ax5, t, double(logical(T.baro_update_used)), "g", "LineWidth", 0.9);
    plot(ax5, t, double(logical(T.baro_rejected)), "r", "LineWidth", 0.9);
    plot(ax5, t, double(logical(T.ekf_ready)), "Color", [0.47 0.67 0.19], "LineWidth", 0.8);
    ylabel(ax5, "Flag");
    ylim(ax5, [-0.05 1.2]);
    add_mask_background(ax5, t, judge_mask);
    xlabel(ax5, "Time since log start [s]");
    title(ax5, "Baro age, update usage, rejection, EKF readiness");
    legend(ax5, "baro age", "update used", "rejected", "ekf ready", "Location", "eastoutside");
    grid(ax5, "on");

    linkaxes([ax1 ax2 ax3 ax4 ax5], "x");
    drawnow;

    if save_plot
        if strlength(string(plot_file)) == 0
            plot_file = replace(string(csv_file), ".CSV", "_baro_height_validation.png");
            plot_file = replace(string(plot_file), ".csv", "_baro_height_validation.png");
        end
        try
            saveas(fig, plot_file);
            fprintf("\nSaved plot: %s\n", string(plot_file));
        catch ME
            warning("Could not save plot: %s", ME.message);
        end
    end
end

%% Overall verdict
hard_fail = false;
warn_count = 0;
for i = 1:numel(results)
    if results{i}.level == "FAIL"
        hard_fail = true;
    elseif results{i}.level == "WARN"
        warn_count = warn_count + 1;
    end
end

fprintf("\n=== Overall verdict ===\n");
if hard_fail
    fprintf("FAIL: baro/vertical-estimation core behavior needs attention.\n");
elseif warn_count > 0
    fprintf("WARN: baro-INS height is usable, but stability/update details need review.\n");
else
    fprintf("PASS: baro-INS height estimation looks acceptable under current limits.\n");
end

fprintf("\nInterpretation guide:\n");
fprintf("- Pressure range failure usually means logging scale/unit bug.\n");
fprintf("- Low update-used or high rejection means EKF is not really using baro reliably.\n");
fprintf("- Large innovation means prediction and baro measurement disagree.\n");
fprintf("- Large EKF height drift with small baro drift points to vertical propagation/accel contamination.\n");
fprintf("- GNSS altitude is not used by this script.\n");

%% Local functions
function assert_required_columns(T, names)
    vars = string(T.Properties.VariableNames);
    missing = names(~ismember(names, vars));
    if ~isempty(missing)
        error("Missing required columns:\n%s", strjoin(missing, newline));
    end
end

function tf = has_column(T, name)
    tf = ismember(string(name), string(T.Properties.VariableNames));
end

function S = empty_stats()
    S.mean = NaN; S.std = NaN; S.rms = NaN; S.p05 = NaN; S.p50 = NaN; S.p95 = NaN;
    S.p95_abs = NaN; S.max = NaN; S.min = NaN; S.max_abs = NaN; S.span = NaN;
end

function S = mag_stats(x, m)
    x = x(m);
    x = x(isfinite(x));
    if isempty(x)
        S = empty_stats();
        return;
    end
    S.mean = mean(x);
    S.std = std(x);
    S.rms = sqrt(mean(x.^2));
    S.p50 = finite_percentile(x, 50);
    S.p95 = finite_percentile(x, 95);
    S.p95_abs = finite_percentile(abs(x), 95);
    S.max = max(x);
    S.min = min(x);
    S.max_abs = max(abs(x));
    S.span = S.max - S.min;
end

function S = signed_stats(x, m)
    S = mag_stats(x, m);
    x = x(m);
    x = x(isfinite(x));
    if isempty(x)
        return;
    end
    S.p05 = finite_percentile(x, 5);
end

function r = finite_mean(x)
    x = x(isfinite(x));
    if isempty(x), r = NaN; else, r = mean(x); end
end

function r = finite_std(x)
    x = x(isfinite(x));
    if isempty(x), r = NaN; else, r = std(x); end
end

function r = finite_median(x)
    x = sort(x(isfinite(x)));
    n = numel(x);
    if n == 0
        r = NaN;
    elseif mod(n, 2) == 1
        r = x((n + 1) / 2);
    else
        r = 0.5 * (x(n / 2) + x(n / 2 + 1));
    end
end

function r = finite_max(x)
    x = x(isfinite(x));
    if isempty(x), r = NaN; else, r = max(x); end
end

function p = finite_percentile(x, q)
    x = sort(x(isfinite(x)));
    n = numel(x);
    if n == 0
        p = NaN;
        return;
    end
    if n == 1
        p = x(1);
        return;
    end
    q = max(0, min(100, q));
    pos = 1 + (n - 1) * q / 100;
    lo = floor(pos);
    hi = ceil(pos);
    if lo == hi
        p = x(lo);
    else
        w = pos - lo;
        p = (1 - w) * x(lo) + w * x(hi);
    end
end

function seg = logical_segments(mask)
    mask = logical(mask(:));
    d = diff([false; mask; false]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
    seg = [starts ends];
end

function dur = mask_duration(t, mask)
    seg = logical_segments(mask);
    dur = 0;
    for k = 1:size(seg, 1)
        dur = dur + max(0, t(seg(k, 2)) - t(seg(k, 1)));
    end
end

function R = check_result(name, pass, detail, fail_msg)
    R.name = string(name);
    if pass
        R.level = "PASS";
        R.message = string(detail);
    else
        R.level = "FAIL";
        R.message = string(fail_msg) + " (" + string(detail) + ")";
    end
end

function R = check_warn(name, pass, detail, warn_msg)
    R.name = string(name);
    if pass
        R.level = "PASS";
        R.message = string(detail);
    else
        R.level = "WARN";
        R.message = string(warn_msg) + " (" + string(detail) + ")";
    end
end

function print_results(results)
    fprintf("--- Validation summary ---\n");
    for i = 1:numel(results)
        R = results{i};
        fprintf("[%s] %-30s %s\n", R.level, R.name + ":", R.message);
    end
end

function print_mag(name, S, unit)
    fprintf("%-18s mean/std/P50/P95/max/span = %.4f / %.4f / %.4f / %.4f / %.4f / %.4f %s\n", ...
        name, S.mean, S.std, S.p50, S.p95, S.max, S.span, unit);
end

function print_signed(name, S, unit)
    fprintf("%-18s mean/std/P05/P95/P95abs/maxabs/span = %+8.4f / %.4f / %+8.4f / %+8.4f / %.4f / %.4f / %.4f %s\n", ...
        name, S.mean, S.std, S.p05, S.p95, S.p95_abs, S.max_abs, S.span, unit);
end

function add_mask_background(ax, t, mask)
    yl = ylim(ax);
    seg = logical_segments(mask);
    for k = 1:size(seg, 1)
        xs = [t(seg(k, 1)) t(seg(k, 2)) t(seg(k, 2)) t(seg(k, 1))];
        ys = [yl(1) yl(1) yl(2) yl(2)];
        h = patch(ax, xs, ys, [0.47 0.67 0.19], ...
            "FaceAlpha", 0.10, "EdgeColor", "none", "HandleVisibility", "off");
        try
            uistack(h, "bottom");
        catch
        end
    end
    ylim(ax, yl);
end
