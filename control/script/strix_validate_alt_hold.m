%% STRIX FC AltHold first-attempt validation
% Validates the first conservative altitude-hold implementation.
%
% Scope:
% - GNSS altitude is intentionally ignored.
% - Height input is baro-INS EKF vertical state converted to height-up:
%       height_m   = -ekf_pos_d
%       vel_up_mps = -ekf_vel_d
% - Checks AltHold enable/allowed/active, target latch, throttle correction,
%   PWM continuity, and baro-INS behavior during active segments.

clear; clc; close all;

%% User settings
csv_file = "";
if strlength(csv_file) == 0
    csv_file = string(getenv("STRIX_ALTHOLD_CSV"));
end
if strlength(csv_file) == 0
    csv_file = "atl_01.CSV";
end
if ~isfile(csv_file)
    fallback = fullfile(pwd, ".cache", "atl_01.CSV");
    if isfile(fallback)
        csv_file = string(fallback);
    else
        [f, p] = uigetfile("*.CSV;*.csv", "Select STRIX FC AltHold log");
        if isequal(f, 0)
            error("No CSV file selected.");
        end
        csv_file = string(fullfile(p, f));
    end
end

% Expected first-test implementation limits.
active_min_duration_s = 2.0;
corr_limit_us = 20.0;
cmd_cap_us = 1450.0;
active_pwm_min_us = 1350.0;
allowed_pwm_min_us = 1300.0;
max_cmd_drop_us = 80.0;        % catches "AltHold button killed throttle" failures
max_latch_error_m = 0.08;      % alt_sp should latch close to active height
max_height_diff_p95_m = 0.80;  % EKF height should stay close to baro measurement
max_innov_p95_m = 0.80;
max_reject_ratio = 0.02;
max_sat_ratio_warn = 0.60;     % saturation is expected if correction limit is tiny, but too much means weak authority

save_figures = false;
fig_dir = "althold_first_attempt_figures";

%% Load
opts = detectImportOptions(csv_file);
opts.VariableNamingRule = "preserve";
T = readtable(csv_file, opts);

required = ["timestamp_ms", ...
    "baro_pressure_pa", "baro_rel_alt_lpf_m", "baro_age_ms", ...
    "baro_ready", "baro_bias_calibrated", "baro_update_used", "baro_rejected", ...
    "ekf_ready", "ekf_pos_d", "ekf_vel_d", ...
    "baro_innov_m", "baro_test_ratio", "baro_height_pred_m", "baro_meas_m", ...
    "ekf_pwm_mean", "motor_output_allowed", "motor_on_detected", "flight_mode", ...
    "roll_deg", "pitch_deg", ...
    "alt_ctrl_enabled", "alt_ctrl_allowed", "alt_ctrl_active", ...
    "alt_ctrl_reset_event", "alt_ctrl_block_reason", "alt_ctrl_dt_s", ...
    "alt_sp_m", "alt_meas_m", "vel_up_mps", "vel_sp_mps", ...
    "alt_err_m", "vel_err_mps", ...
    "alt_p_term_us", "alt_d_term_us", "alt_i_term_us", ...
    "alt_throttle_corr_us", "alt_throttle_corr_sat", ...
    "alt_base_pwm_us", "alt_cmd_pwm_us", ...
    "M1", "M2", "M3", "M4"];
assert_has_vars(T, required);

t = col(T, "timestamp_ms") * 1e-3;
t = t - t(1);
dt = diff(t);
fs_med = 1 / median(dt(dt > 0), "omitnan");

%% Extract
baro_pressure_pa = col(T, "baro_pressure_pa");
baro_rel_lpf = col(T, "baro_rel_alt_lpf_m");
baro_age_ms = col(T, "baro_age_ms");
baro_ready = flag(T, "baro_ready");
baro_bias_cal = flag(T, "baro_bias_calibrated");
baro_update_used = flag(T, "baro_update_used");
baro_rejected = flag(T, "baro_rejected");
baro_innov = col(T, "baro_innov_m");
baro_test_ratio = col(T, "baro_test_ratio");
baro_meas = col(T, "baro_meas_m");
baro_pred = col(T, "baro_height_pred_m");

ekf_ready = flag(T, "ekf_ready");
ekf_pos_d = col(T, "ekf_pos_d");
ekf_vel_d = col(T, "ekf_vel_d");
height_up = -ekf_pos_d;
vel_up_from_ekf = -ekf_vel_d;

ekf_pwm_mean = col(T, "ekf_pwm_mean");
motor_output_allowed = flag(T, "motor_output_allowed");
motor_on = flag(T, "motor_on_detected");
flight_mode = col(T, "flight_mode");
roll_deg = col(T, "roll_deg");
pitch_deg = col(T, "pitch_deg");

alt_enabled = flag(T, "alt_ctrl_enabled");
alt_allowed = flag(T, "alt_ctrl_allowed");
alt_active = flag(T, "alt_ctrl_active");
alt_reset = flag(T, "alt_ctrl_reset_event");
alt_block = col(T, "alt_ctrl_block_reason");
alt_dt_s = col(T, "alt_ctrl_dt_s");
alt_sp = col(T, "alt_sp_m");
alt_meas = col(T, "alt_meas_m");
vel_up = col(T, "vel_up_mps");
vel_sp = col(T, "vel_sp_mps");
alt_err = col(T, "alt_err_m");
vel_err = col(T, "vel_err_mps");
alt_p = col(T, "alt_p_term_us");
alt_d = col(T, "alt_d_term_us");
alt_i = col(T, "alt_i_term_us");
alt_corr = col(T, "alt_throttle_corr_us");
alt_corr_sat = flag(T, "alt_throttle_corr_sat");
alt_base_pwm = col(T, "alt_base_pwm_us");
alt_cmd_pwm = col(T, "alt_cmd_pwm_us");

M = [col(T, "M1"), col(T, "M2"), col(T, "M3"), col(T, "M4")];
motor_mean = mean(M, 2, "omitnan");

%% Masks and events
m_active = alt_active;
m_allowed = alt_allowed;
m_enabled = alt_enabled;
m_ekf_baro = ekf_ready & baro_ready & baro_bias_cal;
m_judge = m_active & m_ekf_baro;

active_segments = get_segments(t, m_active);
allowed_segments = get_segments(t, m_allowed);
enabled_segments = get_segments(t, m_enabled);

active_rise_idx = find(diff([false; m_active]) == 1);
active_fall_idx = find(diff([m_active; false]) == -1);
reset_idx = find(alt_reset);

%% Derived checks
height_diff = height_up - baro_rel_lpf;
meas_pred_diff = baro_meas - baro_pred;
cmd_expected_uncapped = alt_base_pwm + alt_corr;
cmd_cap_error = alt_cmd_pwm - min(cmd_expected_uncapped, cmd_cap_us);

% Look for sudden command loss around AltHold enable/active transitions.
trans_idx = unique([find(diff([alt_enabled(1); alt_enabled]) ~= 0); active_rise_idx; active_fall_idx; reset_idx]);
drop_events = [];
for k = 1:numel(trans_idx)
    i = trans_idx(k);
    i0 = max(1, i-2);
    i1 = min(height(T), i+2);
    local_drop = max(alt_cmd_pwm(i0:i1), [], "omitnan") - min(alt_cmd_pwm(i0:i1), [], "omitnan");
    if local_drop > max_cmd_drop_us
        drop_events(end+1, :) = [i, t(i), local_drop]; %#ok<AGROW>
    end
end

%% Console report
fprintf("\n=== STRIX AltHold First Attempt Validation ===\n");
fprintf("File        : %s\n", csv_file);
fprintf("Rows        : %d\n", height(T));
fprintf("Duration    : %.2f s\n", t(end) - t(1));
fprintf("Median rate : %.2f Hz\n", fs_med);
fprintf("Note        : GNSS altitude is intentionally ignored.\n");

fprintf("\n--- Mode / activation summary ---\n");
fprintf("alt_ctrl_enabled ratio : %6.2f %%\n", 100 * mean(alt_enabled));
fprintf("alt_ctrl_allowed ratio : %6.2f %%\n", 100 * mean(alt_allowed));
fprintf("alt_ctrl_active ratio  : %6.2f %%\n", 100 * mean(alt_active));
fprintf("active duration total  : %.2f s\n", total_duration(active_segments));
fprintf("active segments        : %d\n", size(active_segments, 1));
print_segments("enabled", enabled_segments);
print_segments("allowed", allowed_segments);
print_segments("active", active_segments);

fprintf("\n--- Active transition / latch ---\n");
if isempty(active_rise_idx)
    fprintf("No active rising edge found.\n");
else
    for k = 1:numel(active_rise_idx)
        i = active_rise_idx(k);
        reset_near = any(abs(reset_idx - i) <= 3);
        latch_err = alt_sp(i) - height_up(i);
        fprintf("active rise %d at %.2fs: height %.3f m, alt_sp %.3f m, latch_err %+ .3f m, reset_near=%d, base/cmd/corr %.1f/%.1f/%+.1f us\n", ...
            k, t(i), height_up(i), alt_sp(i), latch_err, reset_near, alt_base_pwm(i), alt_cmd_pwm(i), alt_corr(i));
    end
end

fprintf("\n--- Readiness / baro-INS fusion during active ---\n");
print_ratio("ekf_ready active", ekf_ready, m_active);
print_ratio("baro_ready active", baro_ready, m_active);
print_ratio("baro bias calibrated active", baro_bias_cal, m_active);
print_ratio("baro update used active", baro_update_used, m_active);
print_ratio("baro rejected active", baro_rejected, m_active);
fprintf("baro age active P95/max     : %.1f / %.1f ms\n", pct(baro_age_ms(m_active), 95), max(baro_age_ms(m_active), [], "omitnan"));
fprintf("baro innov active P95/max   : %.3f / %.3f m\n", pct(abs(baro_innov(m_active)), 95), max(abs(baro_innov(m_active)), [], "omitnan"));
fprintf("height-barometer diff P95   : %.3f m\n", pct(abs(height_diff(m_active)), 95));
fprintf("baro test ratio P95/max     : %.4f / %.4f\n", pct(baro_test_ratio(m_active), 95), max(baro_test_ratio(m_active), [], "omitnan"));

fprintf("\n--- Altitude-control output during active ---\n");
print_stats("alt_meas", alt_meas(m_active), "m");
print_stats("alt_sp", alt_sp(m_active), "m");
print_stats("alt_err", alt_err(m_active), "m");
print_stats("vel_up", vel_up(m_active), "m/s");
print_stats("vel_err", vel_err(m_active), "m/s");
print_stats("P term", alt_p(m_active), "us");
print_stats("D term", alt_d(m_active), "us");
print_stats("I term", alt_i(m_active), "us");
print_stats("corr", alt_corr(m_active), "us");
print_stats("base PWM", alt_base_pwm(m_active), "us");
print_stats("cmd PWM", alt_cmd_pwm(m_active), "us");
fprintf("corr saturation active ratio: %.2f %%\n", 100 * mean(alt_corr_sat(m_active)));
fprintf("cmd cap reached ratio       : %.2f %%\n", 100 * mean(alt_cmd_pwm(m_active) >= cmd_cap_us - 1e-3));
fprintf("cmd formula error max abs   : %.3f us\n", max(abs(cmd_cap_error(m_active)), [], "omitnan"));

fprintf("\n--- Block reason values ---\n");
print_value_counts("alt_ctrl_block_reason", alt_block);
fprintf("Block reasons look like a bitmask. Unique values are printed; decode using the FC enum/bit definitions.\n");

fprintf("\n--- Motor command continuity ---\n");
fprintf("motor mean min/mean/max      : %.1f / %.1f / %.1f us\n", min(motor_mean, [], "omitnan"), mean(motor_mean, "omitnan"), max(motor_mean, [], "omitnan"));
fprintf("alt cmd min/mean/max         : %.1f / %.1f / %.1f us\n", min(alt_cmd_pwm, [], "omitnan"), mean(alt_cmd_pwm, "omitnan"), max(alt_cmd_pwm, [], "omitnan"));
if isempty(drop_events)
    fprintf("No >%.1f us command-drop event near AltHold transitions.\n", max_cmd_drop_us);
else
    fprintf("WARNING: command drop events near transitions:\n");
    for k = 1:size(drop_events, 1)
        fprintf("  row %d, t %.2fs, local drop %.1f us\n", drop_events(k, 1), drop_events(k, 2), drop_events(k, 3));
    end
end

fprintf("\n--- Pass / warn checks ---\n");
check_ge("active duration", total_duration(active_segments), active_min_duration_s, "s");
check_ge("active reset event count", numel(reset_idx), 1, "");
if ~isempty(active_rise_idx)
    latch_errors = arrayfun(@(i) abs(alt_sp(i) - height_up(i)), active_rise_idx);
    check_le("max latch error", max(latch_errors), max_latch_error_m, "m");
else
    check_le("max latch error", nan, max_latch_error_m, "m");
end
check_ge("active min cmd PWM", min(alt_cmd_pwm(m_active), [], "omitnan"), active_pwm_min_us - corr_limit_us - 1, "us");
check_le("correction max abs", max(abs(alt_corr(m_active)), [], "omitnan"), corr_limit_us + 1e-3, "us");
check_le("cmd cap max", max(alt_cmd_pwm(m_active), [], "omitnan"), cmd_cap_us + 1e-3, "us");
check_le("cmd formula error", max(abs(cmd_cap_error(m_active)), [], "omitnan"), 1e-3, "us");
check_le("baro rejected active", mean(baro_rejected(m_active)), max_reject_ratio, "");
check_le("baro innovation P95", pct(abs(baro_innov(m_active)), 95), max_innov_p95_m, "m");
check_le("height-baro diff P95", pct(abs(height_diff(m_active)), 95), max_height_diff_p95_m, "m");
check_le("corr saturation ratio", mean(alt_corr_sat(m_active)), max_sat_ratio_warn, "");

fprintf("\nInterpretation notes:\n");
fprintf("- If active never turns on, inspect block_reason and PWM thresholds: allowed > %.0f us, active > %.0f us.\n", allowed_pwm_min_us, active_pwm_min_us);
fprintf("- If alt_sp is not close to height at active rising edge, latch logic is wrong.\n");
fprintf("- If alt_cmd_pwm suddenly drops near AltHold transition, throttle preservation is still broken.\n");
fprintf("- If correction saturates most of the active time, authority is too small or gains/hover PWM are off.\n");
fprintf("- If EKF-barometer difference is small but height moves a lot, the vehicle likely moved; do not judge it as static.\n");

%% Plots
if save_figures && ~isfolder(fig_dir)
    mkdir(fig_dir);
end

fig1 = figure("Name", "AltHold activation and throttle", "Color", "w");
tiledlayout(5, 1, "TileSpacing", "compact");

nexttile;
stairs(t, flight_mode, "k"); hold on;
stairs(t, alt_enabled + 0.05, "b");
stairs(t, alt_allowed + 0.10, "m");
stairs(t, alt_active + 0.15, "g", "LineWidth", 1.1);
stairs(t, alt_reset + 0.20, "r");
grid on; ylim([-0.2 2.4]);
ylabel("mode/flag");
legend("flight mode", "enabled", "allowed", "active", "reset", "Location", "best");
title("AltHold mode and activation flags");

nexttile;
plot(t, alt_base_pwm, "k"); hold on;
plot(t, alt_cmd_pwm, "b", "LineWidth", 1.1);
plot(t, ekf_pwm_mean, "Color", [0.5 0.5 0.5]);
yline(allowed_pwm_min_us, "m--", "allowed");
yline(active_pwm_min_us, "g--", "active");
yline(cmd_cap_us, "r--", "cap");
shade_mask(t, m_active);
grid on; ylabel("PWM [us]");
legend("base", "cmd", "ekf mean", "Location", "best");
title("Throttle command continuity");

nexttile;
plot(t, alt_corr, "b"); hold on;
plot(t, alt_p, "r");
plot(t, alt_d, "m");
plot(t, alt_i, "g");
yline(corr_limit_us, "k--");
yline(-corr_limit_us, "k--");
shade_mask(t, m_active);
grid on; ylabel("us");
legend("corr", "P", "D", "I", "Location", "best");
title("AltHold correction terms");

nexttile;
plot(t, M(:,1), "r"); hold on;
plot(t, M(:,2), "g");
plot(t, M(:,3), "b");
plot(t, M(:,4), "m");
plot(t, motor_mean, "k", "LineWidth", 1.1);
shade_mask(t, m_active);
grid on; ylabel("motor PWM");
legend("M1", "M2", "M3", "M4", "mean", "Location", "best");

nexttile;
plot(t, alt_block, "k");
shade_mask(t, m_active);
grid on; xlabel("time [s]"); ylabel("block reason");
title("AltHold block reason");

fig2 = figure("Name", "AltHold height tracking", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, height_up, "b", "LineWidth", 1.0); hold on;
plot(t, alt_meas, "c");
plot(t, alt_sp, "r", "LineWidth", 1.1);
plot(t, baro_rel_lpf, "k--");
shade_mask(t, m_active);
grid on; ylabel("height [m]");
legend("EKF height=-D", "alt_meas", "alt_sp", "baro rel lpf", "Location", "best");
title("Height target latch and tracking");

nexttile;
plot(t, alt_err, "r"); hold on;
yline(0.5, "k--");
yline(-0.5, "k--");
shade_mask(t, m_active);
grid on; ylabel("alt err [m]");

nexttile;
plot(t, vel_up, "b"); hold on;
plot(t, vel_up_from_ekf, "k--");
plot(t, vel_sp, "r");
shade_mask(t, m_active);
grid on; ylabel("vel [m/s]");
legend("logged vel up", "-ekf vel D", "vel sp", "Location", "best");

nexttile;
plot(t, roll_deg, "r"); hold on;
plot(t, pitch_deg, "b");
yline(25, "k--");
yline(-25, "k--");
shade_mask(t, m_active);
grid on; xlabel("time [s]"); ylabel("deg");
legend("roll", "pitch", "Location", "best");

fig3 = figure("Name", "Baro-INS during AltHold", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");

nexttile;
plot(t, baro_pressure_pa, "k");
shade_mask(t, m_active);
grid on; ylabel("Pa");
title("Barometer pressure");

nexttile;
plot(t, baro_meas, "r"); hold on;
plot(t, baro_pred, "b");
plot(t, meas_pred_diff, "m");
shade_mask(t, m_active);
grid on; ylabel("m");
legend("baro meas", "baro pred", "meas-pred", "Location", "best");

nexttile;
plot(t, baro_innov, "m"); hold on;
plot(t, baro_test_ratio, "k");
shade_mask(t, m_active);
grid on; ylabel("m / ratio");
legend("innovation", "test ratio", "Location", "best");

nexttile;
stairs(t, baro_update_used, "g"); hold on;
stairs(t, baro_rejected, "r");
stairs(t, ekf_ready, "k");
plot(t, baro_age_ms / max(max(baro_age_ms), 1), "b");
shade_mask(t, m_active);
grid on; xlabel("time [s]");
legend("baro update", "baro rejected", "ekf ready", "age norm", "Location", "best");

fig4 = figure("Name", "AltHold scatter checks", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");

nexttile;
scatter(alt_err(m_active), alt_corr(m_active), 18, t(m_active), "filled");
grid on; xlabel("alt err [m]"); ylabel("corr [us]");
title("Height error vs correction"); colorbar;

nexttile;
scatter(vel_err(m_active), alt_corr(m_active), 18, t(m_active), "filled");
grid on; xlabel("vel err [m/s]"); ylabel("corr [us]");
title("Velocity error vs correction"); colorbar;

nexttile;
scatter(alt_base_pwm(m_active), alt_cmd_pwm(m_active), 18, t(m_active), "filled");
grid on; xlabel("base PWM [us]"); ylabel("cmd PWM [us]");
title("Base vs commanded PWM"); colorbar;

nexttile;
scatter(height_up(m_active), baro_rel_lpf(m_active), 18, t(m_active), "filled");
grid on; xlabel("EKF height [m]"); ylabel("baro rel lpf [m]");
title("EKF height vs baro"); colorbar;

if save_figures
    saveas(fig1, fullfile(fig_dir, "01_activation_throttle.png"));
    saveas(fig2, fullfile(fig_dir, "02_height_tracking.png"));
    saveas(fig3, fullfile(fig_dir, "03_baro_ins.png"));
    saveas(fig4, fullfile(fig_dir, "04_scatter.png"));
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

function y = flag(T, name)
y = col(T, name) ~= 0;
end

function p = pct(x, q)
x = x(isfinite(x));
if isempty(x)
    p = nan;
else
    p = prctile(x, q);
end
end

function print_stats(name, x, unit)
x = x(isfinite(x));
if isempty(x)
    fprintf("%-14s: no samples\n", name);
    return;
end
fprintf("%-14s mean/std/P05/P50/P95/min/max = %+8.3f / %7.3f / %+8.3f / %+8.3f / %+8.3f / %+8.3f / %+8.3f %s\n", ...
    name, mean(x), std(x), prctile(x, 5), median(x), prctile(x, 95), min(x), max(x), unit);
end

function print_ratio(name, values, mask)
if ~any(mask)
    fprintf("%-30s: no samples\n", name);
else
    fprintf("%-30s: %6.2f %%\n", name, 100 * mean(values(mask)));
end
end

function seg = get_segments(t, mask)
rise = find(diff([false; mask(:)]) == 1);
fall = find(diff([mask(:); false]) == -1);
seg = zeros(numel(rise), 3);
for i = 1:numel(rise)
    seg(i, :) = [t(rise(i)), t(fall(i)), t(fall(i)) - t(rise(i))];
end
end

function d = total_duration(seg)
if isempty(seg)
    d = 0;
else
    d = sum(seg(:,3));
end
end

function print_segments(name, seg)
if isempty(seg)
    fprintf("%-8s segments: none\n", name);
    return;
end
fprintf("%-8s segments:\n", name);
for i = 1:size(seg, 1)
    fprintf("  %d: %.2f ~ %.2f s, dur %.2f s\n", i, seg(i,1), seg(i,2), seg(i,3));
end
end

function print_value_counts(name, x)
u = unique(x(isfinite(x)));
fprintf("%s:\n", name);
for i = 1:numel(u)
    fprintf("  %g : %d samples\n", u(i), nnz(x == u(i)));
end
end

function check_ge(name, value, limit, unit)
if isnan(value)
    verdict = "N/A";
elseif value >= limit
    verdict = "PASS";
else
    verdict = "WARN";
end
fprintf("[%s] %-28s %8.3f >= %-8.3f %s\n", verdict, name + ":", value, limit, unit);
end

function check_le(name, value, limit, unit)
if isnan(value)
    verdict = "N/A";
elseif value <= limit
    verdict = "PASS";
else
    verdict = "WARN";
end
fprintf("[%s] %-28s %8.3f <= %-8.3f %s\n", verdict, name + ":", value, limit, unit);
end

function shade_mask(t, mask)
yl = ylim;
seg = get_segments(t, mask);
for i = 1:size(seg, 1)
    patch([seg(i,1) seg(i,2) seg(i,2) seg(i,1)], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.85 0.95 0.85], "EdgeColor", "none", "FaceAlpha", 0.25);
end
ylim(yl);
uistack(findobj(gca, "Type", "patch"), "bottom");
end
