clear; clc; close all;

%% STRIX Baro-INS EKF altitude log analysis
% File: alt_ekf_01.CSV
% Purpose:
%   - PWM 증가 시 고도 EKF 흔들림 원인 확인
%   - barometer measurement 문제인지, acceleration contamination인지 구분
%   - N/E/XY/D 가속도 residual 비교

csv_file = "data/vel01.CSV";
if ~isfile(csv_file)
    csv_file = "alt_ekf_01.CSV";
end

T = readtable(csv_file);

fprintf("\n========================================\n");
fprintf("STRIX Baro-INS EKF Altitude Analysis\n");
fprintf("File: %s\n", csv_file);
fprintf("========================================\n");

%% Time
t = double(T.timestamp_ms);
t = (t - t(1)) / 1000.0;

dt = diff(t);
dt_pos = dt(dt > 0);
fs = 1 / median(dt_pos);

fprintf("\n[Time]\n");
fprintf("Samples      : %d\n", height(T));
fprintf("Duration     : %.2f sec\n", t(end));
fprintf("Estimated Fs : %.2f Hz\n", fs);
fprintf("Max dt       : %.3f sec\n", max(dt_pos));

%% Required columns
need = ["ekf_pwm_mean", "ekf_pos_d", "ekf_vel_d", ...
        "baro_meas_m", "baro_height_pred_m", "baro_innov_m", ...
        "baro_R_applied", "baro_NIS", ...
        "raw_acc_norm_g", ...
        "acc_ned_n", "acc_ned_e", "acc_ned_h", "acc_ned_d"];

for i = 1:numel(need)
    if ~ismember(need(i), T.Properties.VariableNames)
        error("Missing column: %s", need(i));
    end
end

%% Basic signals
pwm = double(T.ekf_pwm_mean);

% NED D는 아래 방향 양수일 가능성이 크므로, 보기 편하게 up 방향으로 변환
height_ekf = -double(T.ekf_pos_d);
vel_up     = -double(T.ekf_vel_d);

baro_meas  = double(T.baro_meas_m);
baro_pred  = double(T.baro_height_pred_m);
baro_innov = double(T.baro_innov_m);

R_baro = double(T.baro_R_applied);
NIS    = double(T.baro_NIS);

raw_g = double(T.raw_acc_norm_g);

acc_n = double(T.acc_ned_n);
acc_e = double(T.acc_ned_e);
acc_h = double(T.acc_ned_h);
acc_d = double(T.acc_ned_d);

acc_xy = sqrt(acc_n.^2 + acc_e.^2);

if ismember("baro_update_applied", T.Properties.VariableNames)
    baro_update_applied = double(T.baro_update_applied);
else
    baro_update_applied = nan(size(t));
end

if ismember("baro_update_rejected", T.Properties.VariableNames)
    baro_update_rejected = double(T.baro_update_rejected);
else
    baro_update_rejected = nan(size(t));
end

%% Valid window
idx = true(size(t));

if ismember("ekf_ready", T.Properties.VariableNames)
    idx = idx & (T.ekf_ready > 0);
end

% 초반 안정화 구간 제외
idx = idx & (t > 3.0);

fprintf("\n[Valid window]\n");
fprintf("Used samples : %d / %d\n", sum(idx), numel(idx));

%% Moving residuals
win_baro = max(3, round(2.0 * fs));
win_acc  = max(3, round(1.0 * fs));

height_res = height_ekf - movmean(height_ekf, win_baro, 'omitnan');

baro_innov_res = baro_innov - movmean(baro_innov, win_baro, 'omitnan');

acc_n_res = acc_n - movmean(acc_n, win_acc, 'omitnan');
acc_e_res = acc_e - movmean(acc_e, win_acc, 'omitnan');
acc_d_res = acc_d - movmean(acc_d, win_acc, 'omitnan');

acc_xy_res = sqrt(acc_n_res.^2 + acc_e_res.^2);
raw_g_dev = raw_g - 1.0;

%% PWM bin summary
pwm_bin = round(pwm / 50) * 50;
pwm_list = unique(pwm_bin(idx));
pwm_list = sort(pwm_list);

S = table();

for k = 1:numel(pwm_list)
    p = pwm_list(k);
    id = idx & (pwm_bin == p);

    if sum(id) < 20
        continue;
    end

    row.pwm = p;
    row.samples = sum(id);
    row.duration_sec = sum(id) / fs;

    %% EKF altitude
    row.height_std_m = std(height_ekf(id), 'omitnan');
    row.height_res_std_m = std(height_res(id), 'omitnan');
    row.vel_up_p95_mps = prctile(abs(vel_up(id)), 95);

    %% Barometer update
    row.baro_innov_mean_m = mean(baro_innov(id), 'omitnan');
    row.baro_innov_std_m = std(baro_innov(id), 'omitnan');
    row.baro_innov_p95_m = prctile(abs(baro_innov(id)), 95);
    row.baro_innov_res_std_m = std(baro_innov_res(id), 'omitnan');

    row.R_baro_mean = mean(R_baro(id), 'omitnan');
    row.NIS_mean = mean(NIS(id), 'omitnan');
    row.NIS_p95 = prctile(NIS(id), 95);

    %% Acceleration contamination
    row.raw_g_std = std(raw_g(id), 'omitnan');
    row.raw_g_dev_p95 = prctile(abs(raw_g_dev(id)), 95);

    row.acc_n_res_std = std(acc_n_res(id), 'omitnan');
    row.acc_e_res_std = std(acc_e_res(id), 'omitnan');
    row.acc_xy_res_rms = sqrt(mean(acc_xy_res(id).^2, 'omitnan'));
    row.acc_xy_p95 = prctile(acc_xy(id), 95);

    row.acc_h_rms = sqrt(mean(acc_h(id).^2, 'omitnan'));
    row.acc_d_res_std = std(acc_d_res(id), 'omitnan');

    row.D_over_XY_res_ratio = row.acc_d_res_std / max(row.acc_xy_res_rms, 1e-6);

    if all(~isnan(baro_update_rejected))
        row.reject_ratio = mean(baro_update_rejected(id) > 0, 'omitnan');
    else
        row.reject_ratio = NaN;
    end

    S = [S; struct2table(row)]; %#ok<AGROW>
end

fprintf("\n[PWM bin summary]\n");
disp(S);

%% Correlation
id = idx & isfinite(pwm) ...
         & isfinite(baro_innov) ...
         & isfinite(acc_n_res) ...
         & isfinite(acc_e_res) ...
         & isfinite(acc_d_res) ...
         & isfinite(height_res);

fprintf("\n[Correlation]\n");
fprintf("corr(PWM, abs(baro_innov)) : %.3f\n", local_corr(pwm(id), abs(baro_innov(id))));
fprintf("corr(PWM, abs(acc_n_res))  : %.3f\n", local_corr(pwm(id), abs(acc_n_res(id))));
fprintf("corr(PWM, abs(acc_e_res))  : %.3f\n", local_corr(pwm(id), abs(acc_e_res(id))));
fprintf("corr(PWM, acc_xy_res)      : %.3f\n", local_corr(pwm(id), acc_xy_res(id)));
fprintf("corr(PWM, abs(acc_d_res))  : %.3f\n", local_corr(pwm(id), abs(acc_d_res(id))));
fprintf("corr(PWM, height_res abs)  : %.3f\n", local_corr(pwm(id), abs(height_res(id))));

%% Quick judgment
fprintf("\n[Quick judgment guide]\n");
fprintf("- PWM 증가와 baro_innov_std / NIS가 같이 증가하면: barometer 측정 신뢰도 문제 가능성 큼\n");
fprintf("- PWM 증가와 acc_d_res_std / raw_g_std가 같이 증가하면: vertical acceleration contamination 가능성 큼\n");
fprintf("- acc_xy_res_rms는 작고 acc_d_res_std만 크면: XY보다 D축 오염이 주요 원인\n");
fprintf("- D_over_XY_res_ratio가 1보다 크면: vertical/D축 residual이 XY보다 큼\n");
fprintf("- height_res_std가 PWM 구간에서 커지면: 실제 고도 추정 흔들림 발생\n");

%% Plot 1: altitude overview
figure("Name","Altitude EKF overview");
tiledlayout(4,1);

nexttile;
plot(t, pwm);
grid on;
ylabel("PWM");
title("PWM");

nexttile;
plot(t, height_ekf);
hold on;
plot(t, baro_meas);
grid on;
ylabel("m");
title("Height estimate vs barometer measurement");
legend("EKF height up", "baro meas");

nexttile;
plot(t, vel_up);
grid on;
ylabel("m/s");
title("Vertical velocity up");

nexttile;
plot(t, height_res);
grid on;
ylabel("m");
xlabel("time [s]");
title("EKF height moving residual");

%% Plot 2: barometer update
figure("Name","Barometer update");
tiledlayout(4,1);

nexttile;
plot(t, pwm);
grid on;
ylabel("PWM");
title("PWM");

nexttile;
plot(t, baro_innov);
grid on;
ylabel("m");
title("baro\_innov\_m");

nexttile;
plot(t, R_baro);
grid on;
ylabel("R");
title("baro\_R\_applied");

nexttile;
plot(t, NIS);
grid on;
ylabel("NIS");
xlabel("time [s]");
title("baro\_NIS");

%% Plot 3: acceleration NED comparison
figure("Name","Acceleration NED comparison");
tiledlayout(5,1);

nexttile;
plot(t, pwm);
grid on;
ylabel("PWM");
title("PWM");

nexttile;
plot(t, acc_n_res);
grid on;
ylabel("m/s^2");
title("acc\_ned\_n residual");

nexttile;
plot(t, acc_e_res);
grid on;
ylabel("m/s^2");
title("acc\_ned\_e residual");

nexttile;
plot(t, acc_xy_res);
grid on;
ylabel("m/s^2");
title("XY acceleration residual norm");

nexttile;
plot(t, acc_d_res);
grid on;
ylabel("m/s^2");
xlabel("time [s]");
title("D acceleration residual");

%% Plot 4: raw acceleration and D/XY comparison
figure("Name","Acceleration contamination");
tiledlayout(4,1);

nexttile;
plot(t, pwm);
grid on;
ylabel("PWM");
title("PWM");

nexttile;
plot(t, raw_g);
grid on;
ylabel("g");
title("raw\_acc\_norm\_g");

nexttile;
plot(t, acc_h);
grid on;
ylabel("m/s^2");
title("acc\_ned\_h");

nexttile;
plot(t, acc_d_res);
grid on;
ylabel("m/s^2");
xlabel("time [s]");
title("acc\_ned\_d moving residual");

%% Plot 5: PWM bin metrics
figure("Name","PWM bin metrics");
tiledlayout(4,1);

nexttile;
plot(S.pwm, S.baro_innov_std_m, "-o");
grid on;
xlabel("PWM");
ylabel("m");
title("baro innovation std by PWM");

nexttile;
plot(S.pwm, S.acc_xy_res_rms, "-o");
grid on;
xlabel("PWM");
ylabel("m/s^2");
title("XY acceleration residual RMS by PWM");

nexttile;
plot(S.pwm, S.acc_d_res_std, "-o");
grid on;
xlabel("PWM");
ylabel("m/s^2");
title("D acceleration residual std by PWM");

nexttile;
plot(S.pwm, S.D_over_XY_res_ratio, "-o");
grid on;
xlabel("PWM");
ylabel("ratio");
title("D / XY residual ratio");

fprintf("\nDone.\n");

%% Local functions
function r = local_corr(x, y)
    x = double(x(:));
    y = double(y(:));

    valid = isfinite(x) & isfinite(y);
    x = x(valid);
    y = y(valid);

    if numel(x) < 3
        r = NaN;
        return;
    end

    x = x - mean(x);
    y = y - mean(y);

    den = sqrt(sum(x.^2) * sum(y.^2));

    if den < 1e-12
        r = NaN;
    else
        r = sum(x .* y) / den;
    end
end