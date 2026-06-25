clear; clc; close all;

%% STRIX Outdoor Static Motor-Off Log Check
% File: 1OUTDOOR_STATIC_MOTOROFF.CSV
% Purpose:
%   - 실외 / 기체 고정 / 모터 OFF / 정지 baseline 확인
%   - Barometer drift/noise, IMU 정지 노이즈, GNSS 품질 확인
%   - 다음 PWM sweep 실험으로 넘어가도 되는지 sanity check

%% User settings
csv_file = "data/1OUTDOOR_STATIC_MOTOROFF.CSV";

% 초반 구간은 GNSS/EKF 초기화, baro bias 안정화 가능성이 있으므로 제외
ignore_start_sec = 30.0;

% moving residual 계산용 윈도우
baro_residual_window_sec = 10.0;

%% Load
T = readtable(csv_file);

fprintf("\n========================================\n");
fprintf("STRIX Static Motor-Off Log Check\n");
fprintf("File: %s\n", csv_file);
fprintf("========================================\n");

%% Time
t = double(T.timestamp_ms);
t = (t - t(1)) / 1000.0;

dt = diff(t);
dt = dt(dt > 0);

fs_est = 1 / median(dt);

fprintf("\n[Time]\n");
fprintf("Samples          : %d\n", height(T));
fprintf("Duration         : %.2f sec (%.2f min)\n", t(end), t(end)/60);
fprintf("Estimated Fs     : %.2f Hz\n", fs_est);
fprintf("Median dt        : %.4f sec\n", median(dt));
fprintf("Max dt           : %.4f sec\n", max(dt));

%% Valid analysis index
idx = t >= ignore_start_sec;

if sum(idx) < 100
    error("분석 가능한 샘플이 너무 적음. ignore_start_sec를 줄이세요.");
end

fprintf("\n[Analysis Window]\n");
fprintf("Ignore start     : %.1f sec\n", ignore_start_sec);
fprintf("Used samples     : %d\n", sum(idx));
fprintf("Used duration    : %.2f sec\n", t(find(idx,1,'last')) - t(find(idx,1,'first')));

%% Helper functions
rms_local = @(x) sqrt(mean(x.^2, 'omitnan'));
p95_local = @(x) prctile(abs(x), 95);
range_local = @(x) max(x, [], 'omitnan') - min(x, [], 'omitnan');

%% Motor / state sanity
fprintf("\n[Motor / State Sanity]\n");

motor_cols = ["M1","M2","M3","M4"];
for k = 1:numel(motor_cols)
    c = motor_cols(k);
    if ismember(c, T.Properties.VariableNames)
        x = T.(c);
        fprintf("%s min/mean/max : %.1f / %.1f / %.1f\n", ...
            c, min(x(idx)), mean(x(idx)), max(x(idx)));
    end
end

if ismember("ekf_pwm_mean", T.Properties.VariableNames)
    x = T.ekf_pwm_mean;
    fprintf("ekf_pwm_mean     : %.1f / %.1f / %.1f\n", ...
        min(x(idx)), mean(x(idx)), max(x(idx)));
end

if ismember("ekf_is_armed", T.Properties.VariableNames)
    fprintf("ekf_is_armed max : %.0f\n", max(T.ekf_is_armed(idx)));
end

if ismember("motor_on_detected", T.Properties.VariableNames)
    fprintf("motor_on_detected max : %.0f\n", max(T.motor_on_detected(idx)));
end

%% GNSS quality
fprintf("\n[GNSS Quality]\n");

gnss_cols = ["gnss_fix_type","numSV","gnss_num_sats","gnss_hacc_m","gnss_vacc_m","gnss_sacc_mps","gnss_ref_ready","ekf_ready"];

for k = 1:numel(gnss_cols)
    c = gnss_cols(k);
    if ismember(c, T.Properties.VariableNames)
        x = T.(c);
        fprintf("%-18s min/mean/max : %.3f / %.3f / %.3f\n", ...
            c, min(x(idx),[],'omitnan'), mean(x(idx),'omitnan'), max(x(idx),[],'omitnan'));
    end
end

%% Barometer analysis
fprintf("\n[Barometer]\n");

baro_alt = [];
baro_name = "";

if ismember("baro_rel_alt_m", T.Properties.VariableNames)
    baro_alt = T.baro_rel_alt_m;
    baro_name = "baro_rel_alt_m";
elseif ismember("baro_alt_unfiltered_m", T.Properties.VariableNames)
    baro_alt = T.baro_alt_unfiltered_m;
    baro_name = "baro_alt_unfiltered_m";
else
    warning("barometer altitude column not found.");
end

if ~isempty(baro_alt)
    x = double(baro_alt);
    x_used = x(idx);

    % linear drift 제거 residual
    tt = t(idx);
    p = polyfit(tt, x_used, 1);
    x_trend = polyval(p, tt);
    x_res_linear = x_used - x_trend;

    % moving average residual
    win = max(3, round(baro_residual_window_sec * fs_est));
    x_ma = movmean(x, win, 'omitnan');
    x_res_ma = x - x_ma;

    fprintf("Altitude column  : %s\n", baro_name);
    fprintf("Mean             : %.4f m\n", mean(x_used,'omitnan'));
    fprintf("Std raw          : %.4f m\n", std(x_used,'omitnan'));
    fprintf("Range raw        : %.4f m\n", range_local(x_used));
    fprintf("Drift slope      : %.6f m/s = %.4f m/min\n", p(1), p(1)*60);
    fprintf("Residual std(linear detrend) : %.4f m\n", std(x_res_linear,'omitnan'));
    fprintf("Residual var(linear detrend) : %.6f m^2\n", var(x_res_linear,'omitnan'));
    fprintf("Residual P95(linear detrend) : %.4f m\n", p95_local(x_res_linear));
    fprintf("Residual std(moving avg)     : %.4f m\n", std(x_res_ma(idx),'omitnan'));
    fprintf("Residual var(moving avg)     : %.6f m^2\n", var(x_res_ma(idx),'omitnan'));
end

if ismember("baro_alt_unfiltered_m", T.Properties.VariableNames) && ...
   ismember("baro_rel_alt_m", T.Properties.VariableNames)

    diff_baro = T.baro_alt_unfiltered_m - T.baro_rel_alt_m;

    fprintf("\n[Baro Filter/Bias Difference]\n");
    fprintf("unfiltered - rel mean : %.4f m\n", mean(diff_baro(idx),'omitnan'));
    fprintf("unfiltered - rel std  : %.4f m\n", std(diff_baro(idx),'omitnan'));
    fprintf("unfiltered - rel range: %.4f m\n", range_local(diff_baro(idx)));
end

%% IMU / acceleration analysis
fprintf("\n[IMU / Acceleration]\n");

imu_cols = ["raw_acc_norm_g","accel_norm","acc_ned_h","acc_ned_d","acc_ned_n","acc_ned_e", ...
            "gyro_raw_norm","gyro_lpf_norm","gyro_corrected_norm"];

for k = 1:numel(imu_cols)
    c = imu_cols(k);
    if ismember(c, T.Properties.VariableNames)
        x = double(T.(c));
        fprintf("%-20s mean/std/rms/P95/max : %.5f / %.5f / %.5f / %.5f / %.5f\n", ...
            c, ...
            mean(x(idx),'omitnan'), ...
            std(x(idx),'omitnan'), ...
            rms_local(x(idx)), ...
            p95_local(x(idx)), ...
            max(abs(x(idx)),[],'omitnan'));
    end
end

% Q_acc_z_base 후보: 정지 상태에서 acc_ned_d의 trend 제거 variance
if ismember("acc_ned_d", T.Properties.VariableNames)
    acc_d = double(T.acc_ned_d);
    acc_d_used = acc_d(idx);

    tt = t(idx);
    p_acc = polyfit(tt, acc_d_used, 1);
    acc_d_res = acc_d_used - polyval(p_acc, tt);

    fprintf("\n[Q_acc_z_base candidate]\n");
    fprintf("acc_ned_d residual std : %.6f m/s^2\n", std(acc_d_res,'omitnan'));
    fprintf("acc_ned_d residual var : %.6f (m/s^2)^2\n", var(acc_d_res,'omitnan'));
end

%% EKF vertical state
fprintf("\n[EKF Vertical State]\n");

ekf_cols = ["ekf_pos_d","ekf_vel_d","ekf_p_cov_d","ekf_v_cov_d", ...
            "ekf_innov_pos_d","ekf_innov_vel_d"];

for k = 1:numel(ekf_cols)
    c = ekf_cols(k);
    if ismember(c, T.Properties.VariableNames)
        x = double(T.(c));
        fprintf("%-18s mean/std/P95/max : %.5f / %.5f / %.5f / %.5f\n", ...
            c, ...
            mean(x(idx),'omitnan'), ...
            std(x(idx),'omitnan'), ...
            p95_local(x(idx)), ...
            max(abs(x(idx)),[],'omitnan'));
    end
end

%% Simple pass/fail style comments
fprintf("\n[Quick Judgment]\n");

ok_motor = true;
if ismember("M1", T.Properties.VariableNames)
    ok_motor = ok_motor && all(T.M1(idx) == 1000);
end
if ismember("M2", T.Properties.VariableNames)
    ok_motor = ok_motor && all(T.M2(idx) == 1000);
end
if ismember("M3", T.Properties.VariableNames)
    ok_motor = ok_motor && all(T.M3(idx) == 1000);
end
if ismember("M4", T.Properties.VariableNames)
    ok_motor = ok_motor && all(T.M4(idx) == 1000);
end

if ok_motor
    fprintf("Motor OFF check   : OK, M1~M4 stayed at 1000.\n");
else
    fprintf("Motor OFF check   : WARNING, motor command changed.\n");
end

if ismember("raw_acc_norm_g", T.Properties.VariableNames)
    raw_g_mean = mean(T.raw_acc_norm_g(idx),'omitnan');
    raw_g_std  = std(T.raw_acc_norm_g(idx),'omitnan');

    if abs(raw_g_mean - 1.0) < 0.03 && raw_g_std < 0.02
        fprintf("IMU static check  : OK, raw_acc_norm_g near 1g.\n");
    else
        fprintf("IMU static check  : WARNING, raw_acc_norm_g not very static.\n");
    end
end

if ismember("gnss_fix_type", T.Properties.VariableNames)
    fix_mean = mean(T.gnss_fix_type(idx),'omitnan');
    if fix_mean >= 3
        fprintf("GNSS fix check    : OK, fix_type around 3 or higher.\n");
    else
        fprintf("GNSS fix check    : WARNING, GNSS fix may be weak.\n");
    end
end

if ~isempty(baro_alt)
    if std(x_res_linear,'omitnan') < 0.5
        fprintf("Baro residual     : OK-ish for outdoor windy static baseline.\n");
    else
        fprintf("Baro residual     : WARNING, baro is quite noisy/drifty.\n");
    end
end

fprintf("\nDone.\n");

%% Plots
figure("Name","Motor and GNSS");
tiledlayout(3,1);

nexttile;
plot(t, T.M1, t, T.M2, t, T.M3, t, T.M4);
grid on;
ylabel("PWM");
title("Motor outputs");
legend("M1","M2","M3","M4");

nexttile;
if ismember("gnss_fix_type", T.Properties.VariableNames)
    plot(t, T.gnss_fix_type);
    grid on;
    ylabel("fix type");
    title("GNSS fix type");
end

nexttile;
if ismember("gnss_hacc_m", T.Properties.VariableNames)
    plot(t, T.gnss_hacc_m);
    hold on;
    if ismember("gnss_vacc_m", T.Properties.VariableNames)
        plot(t, T.gnss_vacc_m);
        legend("hAcc","vAcc");
    end
    grid on;
    ylabel("m");
    xlabel("time [s]");
    title("GNSS accuracy");
end

figure("Name","Barometer");
tiledlayout(3,1);

nexttile;
if ismember("baro_alt_unfiltered_m", T.Properties.VariableNames)
    plot(t, T.baro_alt_unfiltered_m);
    grid on;
    ylabel("m");
    title("baro\_alt\_unfiltered\_m");
end

nexttile;
if ismember("baro_rel_alt_m", T.Properties.VariableNames)
    plot(t, T.baro_rel_alt_m);
    grid on;
    ylabel("m");
    title("baro\_rel\_alt\_m");
end

nexttile;
if ~isempty(baro_alt)
    plot(t, x_res_ma);
    grid on;
    ylabel("m");
    xlabel("time [s]");
    title(sprintf("%s residual from %.1f sec moving average", baro_name, baro_residual_window_sec));
end

figure("Name","IMU Static");
tiledlayout(3,1);

nexttile;
if ismember("raw_acc_norm_g", T.Properties.VariableNames)
    plot(t, T.raw_acc_norm_g);
    grid on;
    ylabel("g");
    title("raw\_acc\_norm\_g");
end

nexttile;
if ismember("acc_ned_h", T.Properties.VariableNames)
    plot(t, T.acc_ned_h);
    grid on;
    ylabel("m/s^2");
    title("acc\_ned\_h");
end

nexttile;
if ismember("acc_ned_d", T.Properties.VariableNames)
    plot(t, T.acc_ned_d);
    grid on;
    ylabel("m/s^2");
    xlabel("time [s]");
    title("acc\_ned\_d");
end

figure("Name","EKF Vertical");
tiledlayout(3,1);

nexttile;
if ismember("ekf_pos_d", T.Properties.VariableNames)
    plot(t, T.ekf_pos_d);
    grid on;
    ylabel("m");
    title("ekf\_pos\_d");
end

nexttile;
if ismember("ekf_vel_d", T.Properties.VariableNames)
    plot(t, T.ekf_vel_d);
    grid on;
    ylabel("m/s");
    title("ekf\_vel\_d");
end

nexttile;
if ismember("ekf_innov_pos_d", T.Properties.VariableNames)
    plot(t, T.ekf_innov_pos_d);
    grid on;
    ylabel("m");
    xlabel("time [s]");
    title("ekf\_innov\_pos\_d");
end