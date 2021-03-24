clc; clear;
% I changed the way how torque is calculated

A = [-112.479885835723,-13.7455338822368,4.49919543342893;
    0,0,1;
    1198.72490645226,211.236513337316,-47.9489962580903];
B = [10.5666559417746;0;-112.611366567498];
B = [B/2 B/2];
A2 = zeros(2,2); A2(1,2) = 1;
B2 = zeros(2,2);
B2(2, 1:2) = [-789.735924265072, 789.735924265072];
C = [0 1 0; 0 0 1];
C_img = eye(3);
D_img = zeros(3,2);
C2 = [1 0;0 1];
D = zeros(2,2);

sys1 = ss(A,B,C,D);
sys_img = ss(A,B,C_img, D_img);
sys2 = ss(A2,B2,C2,D);

K = place(A,B,[-27.5 -27.505 -27.495]);
T = feedback(sys_img, K);
dc_g = dcgain(T);
N_bar = 1/dc_g(1,1);

K2 = place(A2,B2,[-147, -141]);
T2 = feedback(sys2, K2);
dc_g2 = dcgain(T2);
N_bar2 = 1/dc_g2(1,1);
%% Assiging values for PID
Kp1 = 0.8;
Ki1 = 0;
Kd1 = 0.002;
N1 = 100;

Kp2 = 1;
Ki2 = 0.00025;
Kd2 = 1e-4;
N2 = 100;
%% Assiging values for SMC
epsilone = -600;
smc1 = [-0.65, 0.2750, 0.15];
N_bar_smc = -3.9;
N_bar_sw = -2.25;

switchAngle = 5*pi/180;
u_max = 45;
%% File sbr_xy_ramp.ttt needs to be opened
clc;
time = 0:0.004:10;
x = [0 0*pi/180 0 0 0];
r_tha = zeros(3, length(time));
r_tha(:,1:end) = 1;
r_sig = zeros(2, length(time));

[x_data_pid, u_data_pid] = sbr_regulator_pid_xy(time, x, K, Kp1, Ki1, Kd1, N1, N_bar, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max);
[x_data_smc, u_data_smc, sigma_smc] = sbr_sliding_mode_xy(time, x, smc1, epsilone, N_bar_smc, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max);
[x_data_sw, u_data_sw, sigma_data_sw] = sbr_final_xy(time, x, K, Kp1, Ki1, Kd1, N1, N_bar, smc1, epsilone, N_bar_sw, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max, switchAngle);
%%
close all;
subplot(2,2,1);hold on;
plot(time(find(time == 2.86):find(time == 5.86)) - 1, x_data_pid(find(time == 2.86):find(time == 5.86),1));
plot(time(find(time == 1.92):find(time == 4.92))-0.06, x_data_smc(find(time == 1.92):find(time == 4.92),1));
plot(time(find(time == 3):find(time == 6))-1.14, x_data_sw(find(time == 3):find(time == 6),1));
ylabel('Linear velocity (m/s)'); lgd = legend('PID', 'SMC', 'Switch');
lgd.NumColumns = 3;
%xlim([0 time(end)]);
hold off;

subplot(2,2,2); hold on;
plot(time(find(time == 5.992):find(time == 8.992))-0.692, x_data_pid(find(time == 5.992):find(time == 8.992),1));
plot(time(find(time == 5.3):find(time == 8.3)), x_data_smc(find(time == 5.3):find(time == 8.3),1));
plot(time(find(time == 6.212):find(time == 9.212))-0.912, x_data_sw(find(time == 6.212):find(time == 9.212),1));
ylabel('Linear velocity (m/s)'); xlim([5.3 8.3]);
hold off;

subplot(2,2,3); hold on;
plot(time(find(time == 2.86):find(time == 5.86)) - 1, x_data_pid(find(time == 2.86):find(time == 5.86),2));
plot(time(find(time == 1.92):find(time == 4.92))-0.06, x_data_smc(find(time == 1.92):find(time == 4.92),2));
plot(time(find(time == 3):find(time == 6))-1.14, x_data_sw(find(time == 3):find(time == 6),2));
patch([1.92 4.92 4.92 1.92], [-5*pi/180, -5*pi/180, 5*pi/180, 5*pi/180], 'green', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);
yline(5*pi/180,'-','SMC', 'LabelVerticalAlignment', 'top' );
yline(5*pi/180,'-','PID', 'LabelVerticalAlignment', 'bottom' );
yline(-5*pi/180,'-');
xlabel('Time (sec)'); ylabel('\theta (rad)');
hold off;

subplot(2,2,4); hold on;
plot(time(find(time == 5.992):find(time == 8.992))-0.692, x_data_pid(find(time == 5.992):find(time == 8.992),2));
plot(time(find(time == 5.3):find(time == 8.3)), x_data_smc(find(time == 5.3):find(time == 8.3),2));
plot(time(find(time == 6.212):find(time == 9.212))-0.912, x_data_sw(find(time == 6.212):find(time == 9.212),2));
patch([5.3 8.3 8.3 5.3], [-5*pi/180, -5*pi/180, 5*pi/180, 5*pi/180], 'green', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);
yline(5*pi/180,'-','SMC', 'LabelVerticalAlignment', 'top' );
yline(5*pi/180,'-','PID', 'LabelVerticalAlignment', 'bottom' );
yline(-5*pi/180,'-');
xlim([5.3 8.3]);
xlabel('Time (sec)'); ylabel('\theta (rad)');
hold off;

%{
figure(3);
plot(time, x_data_pid(:,2), time, x_data_smc(:,2), time, x_data_sw(:,2));
patch([0 time(end) time(end) 0], [-5*pi/180, -5*pi/180, 5*pi/180, 5*pi/180], 'green', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);
yline(5*pi/180,'-','SMC', 'LabelVerticalAlignment', 'top' );
yline(5*pi/180,'-','PID', 'LabelVerticalAlignment', 'bottom' );
yline(-5*pi/180,'-','SMC', 'LabelVerticalAlignment', 'bottom' );
xlabel('Time (sec)'); ylabel('\theta (rad)'); legend('PID', 'SMC', 'Switch');
xlim([0 time(end)]);
%}
