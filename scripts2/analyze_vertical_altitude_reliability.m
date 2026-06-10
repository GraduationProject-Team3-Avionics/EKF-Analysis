%% analyze_vertical_altitude_reliability_with_baro.m
% =================================================
% Vertical Altitude / Reliability Analysis
% - GNSS height + reliability
% - EKF height + EKF internal covariance
% - Barometer height + freshness/age
% - Ctrl start/end 자동 표시
%
% 사용법:
%   1) 이 파일을 CSV가 있는 MATLAB 작업 폴더에 둔다.
%   2) 아래 csv_file만 본인 파일 경로로 수정한다.
%   3) Run
%
% 좌표/부호:
%   - EKF pos_d, ekf_gnss_pos_d는 NED의 D(Down)로 가정
%   - 보기 편하게 height[Up] = -D 로 변환
%
% 주의:
%   - EKF sqrt(P_DD)는 실제 고도 오차가 아니라 필터 내부 추정 불확실도
%   - barometer에 공분산 컬럼이 없으면 baro reliability는 age/fresh/update로만 판단
% =================================================

clear; clc; close all;

%% ===================== User settings =====================
csv_file = "260610\fc_damp_01.CSV";

% 평가 마스크 설정
use_gnss_ref_ready = true;
use_ekf_ready      = true;
use_ctrl_eval_only = false;   % true면 통계 계산을 ctrl 구간만으로 제한

% 그래프 정렬 설정
% true: ctrl start 시점 값을 0으로 맞춰서 변화량 비교
% false: 각 센서/상태가 가진 원래 상대고도 기준 그대로 비교
align_altitudes_at_ctrl_start = true;

% hMSL 상대고도 기준
% "first_valid": valid 첫 샘플 기준
% "ctrl_start": ctrl start 기준
% "none"       : hmsl 원본[m] 그대로, 보통 권장 X
hmsl_reference_mode = "first_valid";

% ctrl 구간 검출용 PWM threshold fallback
pwm_ctrl_threshold_us = 1050;

%% ===================== Load CSV =====================
T = readtable(csv_file, "VariableNamingRule", "preserve");
vars = string(T.Properties.VariableNames);
N = height(T);

fprintf("\n=================================================\n");
fprintf("Vertical Altitude / Reliability Analysis\n");
fprintf("=================================================\n");
fprintf("CSV        : %s\n", csv_file);
fprintf("Rows       : %d\n", N);
fprintf("Columns    : %d\n", width(T));

%% ===================== Time =====================
time_col = find_col(vars, ["timestamp_ms", "time_ms", "t_ms", "timestamp", "time"]);
if time_col == ""
    error("No timestamp column found.");
end

t_raw = get_col(T, time_col);
if contains(lower(time_col), "ms") || max(t_raw, [], "omitnan") > 1000
    t = (t_raw - t_raw(1)) * 1e-3;
else
    t = t_raw - t_raw(1);
end

%% ===================== Basic masks =====================
valid = true(N, 1);

if use_ekf_ready && has_col(vars, "ekf_ready")
    valid = valid & logical(get_col(T, "ekf_ready"));
end

if use_gnss_ref_ready && has_col(vars, "gnss_ref_ready")
    valid = valid & logical(get_col(T, "gnss_ref_ready"));
end

if has_col(vars, "gnss_valid")
    valid_gnss = logical(get_col(T, "gnss_valid"));
else
    valid_gnss = true(N, 1);
end

valid = valid & isfinite(t);

%% ===================== Ctrl start/end detection =====================
[ctrl_mask, ctrl_col] = detect_ctrl_mask(T, vars, pwm_ctrl_threshold_us);
ctrl_idx = find(ctrl_mask & isfinite(t));

if isempty(ctrl_idx)
    idx_start = find(valid, 1, "first");
    idx_end   = find(valid, 1, "last");
    ctrl_found = false;
    fprintf("Ctrl column: not found. Using valid range for start/end markers.\n");
else
    idx_start = ctrl_idx(1);
    idx_end   = ctrl_idx(end);
    ctrl_found = true;
    fprintf("Ctrl column: %s\n", ctrl_col);
    fprintf("Ctrl range : %.3f ~ %.3f sec\n", t(idx_start), t(idx_end));
end

if isempty(idx_start) || isempty(idx_end)
    error("No valid samples found for analysis.");
end

eval_mask = valid;
if use_ctrl_eval_only
    eval_mask = eval_mask & ctrl_mask;
end

fprintf("Valid eval samples: %d / %d\n", nnz(eval_mask), N);

%% ===================== Column detection =====================
% GNSS hMSL
gnss_hmsl_col = find_col(vars, ["hmsl", "gnss_hmsl", "gnss_hmsl_m", "height_msl", "gnss_height_m"]);
has_gnss_hmsl = gnss_hmsl_col ~= "";

% GNSS vertical accuracy / sigma
gnss_vacc_col = find_col(vars, ["gnss_vacc_m", "vacc_m", "vAcc", "gnss_vertical_accuracy_m", "gnss_alt_acc_m"]);
has_gnss_vacc = gnss_vacc_col ~= "";

% EKF D position
ekf_d_col = find_col(vars, ["ekf_pos_d", "pos_d", "ekf_d"]);
has_ekf_d = ekf_d_col ~= "";

% EKF covariance D
ekf_p_d_col = find_col(vars, ["ekf_p_cov_d", "ekf_P_pos_d", "ekf_pos_d_cov", "p_cov_d", "P_DD"]);
has_ekf_p_d = ekf_p_d_col ~= "";

% GNSS EKF input D
gnss_input_d_col = find_col(vars, ["ekf_gnss_pos_d", "gnss_pos_d", "gnss_d"]);
has_gnss_input_d = gnss_input_d_col ~= "";

% Applied GNSS D R/sigma
gnss_R_d_col = find_col(vars, ["ekf_R_applied_gnss_pos_d", "R_gnss_pos_d", "gnss_R_pos_d"]);
has_gnss_R_d = gnss_R_d_col ~= "";

gnss_sigma_d_col = find_col(vars, ["ekf_sigma_applied_gnss_pos_d", "sigma_gnss_pos_d", "gnss_sigma_pos_d"]);
has_gnss_sigma_d = gnss_sigma_d_col ~= "";

% Barometer altitude
baro_alt_candidates = ["baro_rel_alt_m", "baro_abs_alt_m", ...
                       "baro_alt_m", "baro_altitude_m", "barometer_alt_m", ...
                       "baro_height_m", "baro_h_m", "baro_pos_d", "baro_d", ...
                       "pressure_alt_m", "baro_alt"];
baro_col = find_col(vars, baro_alt_candidates);
has_baro = baro_col ~= "";

% Barometer status columns
baro_age_col     = find_col(vars, ["baro_age_ms", "baro_age", "barometer_age_ms"]);
baro_fresh_col   = find_col(vars, ["baro_fresh", "baro_is_fresh"]);
baro_updated_col = find_col(vars, ["baro_updated", "barometer_updated"]);
baro_bias_col    = find_col(vars, ["baro_alt_bias_m", "baro_bias_m"]);
baro_press_col   = find_col(vars, ["baro_pressure_pa", "pressure_pa"]);
baro_temp_col    = find_col(vars, ["baro_temp_c", "barometer_temp_c"]);

if has_baro
    fprintf("Barometer column: %s\n", baro_col);
else
    fprintf("Barometer column: not found. Barometer plot will be skipped.\n");
    fprintf("Tried names: %s\n", strjoin(baro_alt_candidates, ", "));
end

%% ===================== Build altitude signals =====================
signals = struct();

% GNSS hMSL: 보통 mm 단위일 수 있으므로 자동 변환
if has_gnss_hmsl
    hmsl_raw = get_col(T, gnss_hmsl_col);
    if median(abs(hmsl_raw), "omitnan") > 10000
        hmsl_m = hmsl_raw * 1e-3;   % mm -> m
    else
        hmsl_m = hmsl_raw;
    end

    switch lower(string(hmsl_reference_mode))
        case "first_valid"
            ref_idx = find(valid_gnss & isfinite(hmsl_m), 1, "first");
            if isempty(ref_idx), ref_idx = find(isfinite(hmsl_m), 1, "first"); end
            gnss_hmsl_rel = hmsl_m - hmsl_m(ref_idx);
        case "ctrl_start"
            gnss_hmsl_rel = hmsl_m - hmsl_m(idx_start);
        case "none"
            gnss_hmsl_rel = hmsl_m;
        otherwise
            error("Unknown hmsl_reference_mode: %s", hmsl_reference_mode);
    end

    signals.gnss_hmsl.name = "GNSS hMSL relative height";
    signals.gnss_hmsl.y    = gnss_hmsl_rel;
    signals.gnss_hmsl.ok   = valid_gnss & isfinite(gnss_hmsl_rel);
end

if has_ekf_d
    ekf_height = -get_col(T, ekf_d_col);
    signals.ekf.name = "EKF height -ekf_pos_d";
    signals.ekf.y    = ekf_height;
    signals.ekf.ok   = valid & isfinite(ekf_height);
end

if has_gnss_input_d
    gnss_input_height = -get_col(T, gnss_input_d_col);
    signals.gnss_input.name = "GNSS EKF-input height -ekf_gnss_pos_d";
    signals.gnss_input.y    = gnss_input_height;
    signals.gnss_input.ok   = valid_gnss & isfinite(gnss_input_height);
end

if has_baro
    baro_alt = get_col(T, baro_col);

    % baro_pos_d, baro_d처럼 D축이면 Up height로 부호 변환
    if any(strcmpi(baro_col, ["baro_pos_d", "baro_d"]))
        baro_height = -baro_alt;
    else
        baro_height = baro_alt;
    end

    signals.baro.name = "Barometer height " + string(baro_col);
    signals.baro.y    = baro_height;

    baro_ok = isfinite(baro_height);
    if baro_fresh_col ~= ""
        % fresh가 있으면 너무 오래된 값은 통계에서 제외 가능
        % 단, 그래프에는 전체를 보되 통계 mask에 fresh 반영
        baro_ok = baro_ok & logical(get_col(T, baro_fresh_col));
    end
    signals.baro.ok = baro_ok;
end

%% ===================== Optional alignment at ctrl start =====================
plot_signals = signals;
if align_altitudes_at_ctrl_start
    fns = fieldnames(plot_signals);
    for i = 1:numel(fns)
        fn = fns{i};
        y = plot_signals.(fn).y;
        ref = y(idx_start);
        if ~isfinite(ref)
            ref_idx = find(plot_signals.(fn).ok & eval_mask, 1, "first");
            if ~isempty(ref_idx)
                ref = y(ref_idx);
            else
                ref = 0;
            end
        end
        plot_signals.(fn).y = y - ref;
        plot_signals.(fn).name = plot_signals.(fn).name + " | aligned at ctrl start";
    end
end

%% ===================== Numeric summaries =====================
fprintf("-------------------------------------------------\n");
print_signal_stats("GNSS hMSL relative height [m]", get_signal(signals, "gnss_hmsl"), eval_mask);
print_signal_stats("EKF height -ekf_pos_d [m]", get_signal(signals, "ekf"), eval_mask);
print_signal_stats("GNSS EKF-input height -ekf_gnss_pos_d [m]", get_signal(signals, "gnss_input"), eval_mask);
print_signal_stats("Barometer height [m]", get_signal(signals, "baro"), eval_mask);

if has_gnss_input_d && has_ekf_d
    diff_gnss_ekf = signals.gnss_input.y - signals.ekf.y;
    print_stats("GNSS EKF-input - EKF [m]", diff_gnss_ekf, eval_mask & signals.gnss_input.ok & signals.ekf.ok);
end

if has_baro && has_ekf_d
    diff_baro_ekf = signals.baro.y - signals.ekf.y;
    print_stats("Barometer - EKF [m]", diff_baro_ekf, eval_mask & signals.baro.ok & signals.ekf.ok);
end

if has_baro && has_gnss_input_d
    diff_baro_gnss = signals.baro.y - signals.gnss_input.y;
    print_stats("Barometer - GNSS EKF-input [m]", diff_baro_gnss, eval_mask & signals.baro.ok & signals.gnss_input.ok);
end

if has_gnss_vacc
    gnss_vacc = get_col(T, gnss_vacc_col);
    print_stats("GNSS vertical sigma/vAcc [m]", gnss_vacc, eval_mask & valid_gnss & isfinite(gnss_vacc));
    print_stats("GNSS vertical covariance vAcc^2 [m^2]", gnss_vacc.^2, eval_mask & valid_gnss & isfinite(gnss_vacc));
end

if has_gnss_sigma_d
    sig_applied_d = get_col(T, gnss_sigma_d_col);
    print_stats("Applied GNSS D sigma in EKF [m]", sig_applied_d, eval_mask & isfinite(sig_applied_d));
end

if has_gnss_R_d
    R_applied_d = get_col(T, gnss_R_d_col);
    print_stats("Applied GNSS D R in EKF [m^2]", R_applied_d, eval_mask & isfinite(R_applied_d));
end

if has_ekf_p_d
    P_DD = get_col(T, ekf_p_d_col);
    sigma_ekf_d = sqrt(max(P_DD, 0));
    print_stats("EKF vertical sigma sqrt(P_DD) [m]", sigma_ekf_d, eval_mask & isfinite(sigma_ekf_d));
end

if baro_age_col ~= ""
    baro_age = get_col(T, baro_age_col);
    print_stats("Barometer age [ms]", baro_age, eval_mask & isfinite(baro_age));
end

fprintf("-------------------------------------------------\n");
if ctrl_found
    ctrl_eval = eval_mask & ctrl_mask;
    fprintf("[During ctrl only]\n");

    if has_gnss_input_d && has_ekf_d
        print_stats("GNSS EKF-input - EKF [m]", diff_gnss_ekf, ctrl_eval & signals.gnss_input.ok & signals.ekf.ok);
    end
    if has_baro && has_ekf_d
        print_stats("Barometer - EKF [m]", diff_baro_ekf, ctrl_eval & signals.baro.ok & signals.ekf.ok);
    end
    if has_baro && has_gnss_input_d
        print_stats("Barometer - GNSS EKF-input [m]", diff_baro_gnss, ctrl_eval & signals.baro.ok & signals.gnss_input.ok);
    end
    if has_gnss_vacc
        print_stats("GNSS vertical sigma/vAcc [m]", gnss_vacc, ctrl_eval & valid_gnss & isfinite(gnss_vacc));
    end
    if has_gnss_R_d
        print_stats("Applied GNSS D R in EKF [m^2]", R_applied_d, ctrl_eval & isfinite(R_applied_d));
    end
    if has_ekf_p_d
        print_stats("EKF vertical sigma sqrt(P_DD) [m]", sigma_ekf_d, ctrl_eval & isfinite(sigma_ekf_d));
    end
    if baro_age_col ~= ""
        print_stats("Barometer age [ms]", baro_age, ctrl_eval & isfinite(baro_age));
    end
end

%% ===================== Figure 1: Altitude comparison =====================
figure("Name", "Vertical altitude comparison");
hold on; grid on;

legend_entries = strings(0);

if isfield(plot_signals, "gnss_hmsl")
    plot(t, plot_signals.gnss_hmsl.y, "LineWidth", 1.1);
    legend_entries(end+1) = plot_signals.gnss_hmsl.name;
end

if isfield(plot_signals, "gnss_input")
    plot(t, plot_signals.gnss_input.y, "LineWidth", 1.1);
    legend_entries(end+1) = plot_signals.gnss_input.name;
end

if isfield(plot_signals, "ekf")
    plot(t, plot_signals.ekf.y, "LineWidth", 1.3);
    legend_entries(end+1) = plot_signals.ekf.name;
end

if isfield(plot_signals, "baro")
    plot(t, plot_signals.baro.y, "LineWidth", 1.1);
    legend_entries(end+1) = plot_signals.baro.name;
end

add_ctrl_lines(t, idx_start, idx_end);
xlabel("Time [s]");
ylabel("Height Up [m]");
title("GNSS / EKF / Barometer altitude comparison");
legend(legend_entries, "Location", "best");

%% ===================== Figure 2: Altitude difference =====================
figure("Name", "Vertical altitude differences");
tiledlayout(3,1, "TileSpacing", "compact");

nexttile; hold on; grid on;
if has_gnss_input_d && has_ekf_d
    plot(t, diff_gnss_ekf, "LineWidth", 1.1);
    ylabel("[m]");
    title("GNSS EKF-input - EKF");
else
    text(0.1, 0.5, "GNSS input or EKF height not available");
end
add_ctrl_lines(t, idx_start, idx_end);

nexttile; hold on; grid on;
if has_baro && has_ekf_d
    plot(t, diff_baro_ekf, "LineWidth", 1.1);
    ylabel("[m]");
    title("Barometer - EKF");
else
    text(0.1, 0.5, "Barometer or EKF height not available");
end
add_ctrl_lines(t, idx_start, idx_end);

nexttile; hold on; grid on;
if has_baro && has_gnss_input_d
    plot(t, diff_baro_gnss, "LineWidth", 1.1);
    ylabel("[m]");
    title("Barometer - GNSS EKF-input");
else
    text(0.1, 0.5, "Barometer or GNSS input height not available");
end
add_ctrl_lines(t, idx_start, idx_end);
xlabel("Time [s]");

%% ===================== Figure 3: Reliability / covariance =====================
figure("Name", "Vertical reliability and covariance");
tiledlayout(3,1, "TileSpacing", "compact");

nexttile; hold on; grid on;
leg = strings(0);
if has_gnss_vacc
    plot(t, gnss_vacc, "LineWidth", 1.1);
    leg(end+1) = "GNSS vAcc / vertical sigma";
end
if has_gnss_sigma_d
    plot(t, sig_applied_d, "LineWidth", 1.1);
    leg(end+1) = "EKF applied GNSS D sigma";
elseif has_gnss_R_d
    plot(t, sqrt(max(R_applied_d,0)), "LineWidth", 1.1);
    leg(end+1) = "sqrt(EKF applied GNSS D R)";
end
if has_ekf_p_d
    plot(t, sigma_ekf_d, "LineWidth", 1.1);
    leg(end+1) = "EKF sqrt(P_DD)";
end
ylabel("Sigma [m]");
title("Vertical sigma comparison");
if ~isempty(leg), legend(leg, "Location", "best"); end
add_ctrl_lines(t, idx_start, idx_end);

nexttile; hold on; grid on;
leg = strings(0);
if has_gnss_vacc
    plot(t, gnss_vacc.^2, "LineWidth", 1.1);
    leg(end+1) = "GNSS vAcc^2";
end
if has_gnss_R_d
    plot(t, R_applied_d, "LineWidth", 1.1);
    leg(end+1) = "EKF applied GNSS D R";
end
if has_ekf_p_d
    plot(t, P_DD, "LineWidth", 1.1);
    leg(end+1) = "EKF P_DD";
end
ylabel("Variance [m^2]");
title("Vertical variance / covariance comparison");
if ~isempty(leg), legend(leg, "Location", "best"); end
add_ctrl_lines(t, idx_start, idx_end);

nexttile; hold on; grid on;
leg = strings(0);
if baro_age_col ~= ""
    plot(t, baro_age, "LineWidth", 1.1);
    leg(end+1) = "baro age [ms]";
end
if baro_fresh_col ~= ""
    plot(t, double(logical(get_col(T, baro_fresh_col))) * 100, "LineWidth", 1.1);
    leg(end+1) = "baro fresh x100";
end
if baro_updated_col ~= ""
    plot(t, double(logical(get_col(T, baro_updated_col))) * 100, "LineWidth", 1.1);
    leg(end+1) = "baro updated x100";
end
if isempty(leg)
    text(0.1, 0.5, "No barometer status columns found");
end
ylabel("Baro status");
xlabel("Time [s]");
title("Barometer freshness / age");
if ~isempty(leg), legend(leg, "Location", "best"); end
add_ctrl_lines(t, idx_start, idx_end);

%% ===================== Figure 4: Barometer raw status =====================
if has_baro
    figure("Name", "Barometer raw data");
    tiledlayout(4,1, "TileSpacing", "compact");

    nexttile; hold on; grid on;
    plot(t, signals.baro.y, "LineWidth", 1.1);
    ylabel("Height [m]");
    title("Barometer height");
    add_ctrl_lines(t, idx_start, idx_end);

    nexttile; hold on; grid on;
    if baro_press_col ~= ""
        plot(t, get_col(T, baro_press_col), "LineWidth", 1.1);
        ylabel("Pressure [Pa]");
        title("Barometer pressure");
    else
        text(0.1, 0.5, "No baro_pressure_pa column");
    end
    add_ctrl_lines(t, idx_start, idx_end);

    nexttile; hold on; grid on;
    if baro_temp_col ~= ""
        plot(t, get_col(T, baro_temp_col), "LineWidth", 1.1);
        ylabel("Temp [degC]");
        title("Barometer temperature");
    else
        text(0.1, 0.5, "No baro_temp_c column");
    end
    add_ctrl_lines(t, idx_start, idx_end);

    nexttile; hold on; grid on;
    if baro_bias_col ~= ""
        plot(t, get_col(T, baro_bias_col), "LineWidth", 1.1);
        ylabel("Bias [m]");
        title("Barometer altitude bias");
    else
        text(0.1, 0.5, "No baro_alt_bias_m column");
    end
    xlabel("Time [s]");
    add_ctrl_lines(t, idx_start, idx_end);
end

%% ===================== Figure 5: Vertical velocity / acceleration sanity =====================
figure("Name", "Vertical EKF sanity check");
tiledlayout(3,1, "TileSpacing", "compact");

nexttile; hold on; grid on;
if has_col(vars, "ekf_vel_d")
    ekf_vz_up = -get_col(T, "ekf_vel_d");
    plot(t, ekf_vz_up, "LineWidth", 1.1);
    ylabel("V_z Up [m/s]");
    title("EKF vertical velocity");
else
    text(0.1, 0.5, "No ekf_vel_d column");
end
add_ctrl_lines(t, idx_start, idx_end);

nexttile; hold on; grid on;
if has_col(vars, "ekf_innov_pos_d")
    innov_h = -get_col(T, "ekf_innov_pos_d");
    plot(t, innov_h, "LineWidth", 1.1);
    ylabel("Innovation height [m]");
    title("GNSS vertical innovation converted to Up sign");
else
    text(0.1, 0.5, "No ekf_innov_pos_d column");
end
add_ctrl_lines(t, idx_start, idx_end);

nexttile; hold on; grid on;
if has_col(vars, "acc_ned_d")
    acc_up = -get_col(T, "acc_ned_d");
    plot(t, acc_up, "LineWidth", 1.1);
    ylabel("Acc Up [m/s^2]");
    title("Vertical acceleration sanity check");
else
    text(0.1, 0.5, "No acc_ned_d column");
end
xlabel("Time [s]");
add_ctrl_lines(t, idx_start, idx_end);

%% ===================== Finish =====================
fprintf("=================================================\n");
fprintf("Done.\n");
fprintf("Notes:\n");
fprintf("  1) EKF sqrt(P_DD) is filter-estimated uncertainty, not true altitude error.\n");
fprintf("  2) GNSS vAcc^2 is a useful starting point for GNSS vertical R.\n");
fprintf("  3) If barometer has no covariance column, judge it using Baro-EKF/GNSS differences, age, fresh, and pressure/temperature behavior.\n");
fprintf("=================================================\n\n");

%% =================================================
% Local functions
% =================================================

function tf = has_col(vars, name)
    tf = any(strcmpi(vars, string(name)));
end

function col = find_col(vars, candidates)
    col = "";
    candidates = string(candidates);
    for k = 1:numel(candidates)
        idx = find(strcmpi(vars, candidates(k)), 1);
        if ~isempty(idx)
            col = vars(idx);
            return;
        end
    end
end

function x = get_col(T, name)
    name = string(name);
    vars = string(T.Properties.VariableNames);
    idx = find(strcmpi(vars, name), 1);
    if isempty(idx)
        error("Column not found: %s", name);
    end
    x = T{:, idx};

    if iscell(x)
        x = str2double(string(x));
    elseif isstring(x) || ischar(x)
        x = str2double(string(x));
    elseif islogical(x)
        x = double(x);
    else
        x = double(x);
    end
end

function [ctrl_mask, ctrl_col] = detect_ctrl_mask(T, vars, pwm_threshold)
    N = height(T);
    ctrl_mask = false(N, 1);
    ctrl_col = "";

    priority = ["vel_ctrl_valid", ...
                "vel_ctrl_enabled", ...
                "motor_output_allowed", ...
                "ekf_is_armed", ...
                "motor_on_detected"];

    for k = 1:numel(priority)
        if any(strcmpi(vars, priority(k)))
            v = get_col(T, priority(k));
            if any(v > 0)
                ctrl_mask = logical(v > 0);
                ctrl_col = priority(k);
                return;
            end
        end
    end

    if any(strcmpi(vars, "ekf_pwm_mean"))
        pwm = get_col(T, "ekf_pwm_mean");
        ctrl_mask = pwm > pwm_threshold;
        ctrl_col = "ekf_pwm_mean > " + string(pwm_threshold);
        if any(ctrl_mask)
            return;
        end
    end

    motor_cols = ["M1", "M2", "M3", "M4"];
    if all(ismember(lower(motor_cols), lower(vars)))
        pwm_mean = zeros(N, 1);
        for i = 1:numel(motor_cols)
            pwm_mean = pwm_mean + get_col(T, motor_cols(i));
        end
        pwm_mean = pwm_mean / numel(motor_cols);
        ctrl_mask = pwm_mean > pwm_threshold;
        ctrl_col = "mean(M1:M4) > " + string(pwm_threshold);
        if any(ctrl_mask)
            return;
        end
    end

    ctrl_mask = false(N, 1);
end

function s = get_signal(signals, field_name)
    if isfield(signals, field_name)
        s = signals.(field_name).y;
    else
        s = [];
    end
end

function print_signal_stats(label, y, mask)
    if isempty(y)
        fprintf("%-50s : not available\n", label);
        return;
    end
    print_stats(label, y, mask & isfinite(y));
end

function print_stats(label, y, mask)
    y = y(:);
    mask = mask(:) & isfinite(y);

    if nnz(mask) == 0
        fprintf("%-50s : no valid samples\n", label);
        return;
    end

    yy = y(mask);
    fprintf("%-50s : mean=%9.4f  std=%9.4f  rms=%9.4f  p95=%9.4f  min=%9.4f  max=%9.4f\n", ...
        label, ...
        mean(yy, "omitnan"), ...
        std(yy, "omitnan"), ...
        sqrt(mean(yy.^2, "omitnan")), ...
        prctile(abs(yy), 95), ...
        min(yy, [], "omitnan"), ...
        max(yy, [], "omitnan"));
end

function add_ctrl_lines(t, idx_start, idx_end)
    if ~isempty(idx_start) && ~isempty(idx_end) && isfinite(t(idx_start)) && isfinite(t(idx_end))
        xline(t(idx_start), "--", "ctrl start", "LabelVerticalAlignment", "bottom");
        xline(t(idx_end), "--", "ctrl end", "LabelVerticalAlignment", "bottom");
    end
end
