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
%% Recovery from 55 deg IC
clc;
time = 0:0.004:2.2;
x = [0 55*pi/180 0 0 0];
r_tha = zeros(3, length(time));
r_sig = zeros(2, length(time));

%[x_data_pid, u_data_pid] = sbr_regulator_pid_xy(time, x, K, Kp1, Ki1, Kd1, N1, N_bar, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max);
%[x_data_smc, u_data_smc, sigma_smc] = sbr_sliding_mode_xy(time, x, smc1, epsilone, N_bar_smc, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max);
%[x_data_sw, u_data_sw, sigma_data_sw] = sbr_final_xy(time, x, K, Kp1, Ki1, Kd1, N1, N_bar, smc1, epsilone, N_bar_sw, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max, switchAngle);
[x_data_sw2, u_data_sw2, sigma_data_sw2] = sbr_final_xy_with_avg(time, x, K, Kp1, Ki1, Kd1, N1, N_bar, smc1, epsilone, N_bar_sw, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max, switchAngle);
%%
plot(time, u_data_sw(:,7), time, u_data_sw2(:,7))


%%
figure(1);
plot(time, x_data_pid(:,1), time, x_data_smc(:,1), time, x_data_sw(:,1));
xlabel('Time (sec)'); ylabel('Linear velocity (m/s)'); legend('PID', 'SMC', 'Switch');
xlim([0 time(end)]);

dist_pid = trapz(time, x_data_pid(:,1)); dist_smc = trapz(time, x_data_smc(:,1)); dist_sw = trapz(time, x_data_sw(:,1));
txt = sprintf('Distance traveled: PID - %2.2f, SMC - %2.2f, SW - %2.2f', dist_pid, dist_smc, dist_sw);
%annotation('textbox',[0.3 0.9 .1 .1],'String', txt, 'EdgeColor','k')
%%
figure(2);
plot(time, x_data_pid(:,2), time, x_data_smc(:,2), time, x_data_sw(:,2));
%patch([0 time(end) time(end) 0], [-5*pi/180, -5*pi/180, 5*pi/180, 5*pi/180], 'green', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);
yline(5*pi/180,'--','SMC', 'LabelVerticalAlignment', 'top' );
yline(5*pi/180,'--','PID', 'LabelVerticalAlignment', 'bottom', 'Alpha', 0 );
yline(-5*pi/180,'--','SMC', 'LabelVerticalAlignment', 'bottom' );
xlabel('Time (sec)'); ylabel('\theta (rad)'); legend('PID', 'SMC', 'Switch');
xlim([0 time(end)]);
%%
figure(3);
plot(time, u_data_pid(:,5), time, u_data_smc(:,5), time, u_data_sw(:,7));
xlabel('Time (sec)'); ylabel('Input variable, u (Volts)'); legend('PID', 'SMC', 'Switch');
xlim([0 time(end)]);
e_pid = trapz(time, abs(u_data_pid(:,1))); e_smc = trapz(time, abs(u_data_smc(:,1))); e_sw = trapz(time, abs(u_data_sw(:,5)));
txt = sprintf('Energy consumed: PID - %2.2f, SMC - %2.2f, SW - %2.2f', e_pid, e_smc, e_sw);
%annotation('textbox',[0.3 0.9 .1 .1],'String', txt, 'EdgeColor','k')
%% Recovery from 25 deg IC
clc;
time = 0:0.004:2;
x = [0 25*pi/180 0 0 0];
r_tha = zeros(3, length(time));
r_sig = zeros(2, length(time));
u_max = 45;
switchingAngle = 5*pi/180;

[x_data_pid, u_data_pid] = sbr_regulator_pid_xy(time, x, K, Kp1, Ki1, Kd1, N1, N_bar, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max);
[x_data_smc, u_data_smc, sigma_smc] = sbr_sliding_mode_xy(time, x, smc1, epsilone, N_bar_smc, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max);
[x_data_sw, u_data_sw, sigma_data_sw] = sbr_final_xy(time, x, K, Kp1, Ki1, Kd1, N1, N_bar, smc1, epsilone, N_bar_sw, r_tha, K2, Kp2, Ki2, Kd2, N2, N_bar2, r_sig, u_max, switchAngle);
%%
figure(1);
plot(time, x_data_pid(:,1), time, x_data_smc(:,1), time, x_data_sw(:,1));
xlabel('Time (sec)'); ylabel('Linear velocity (m/s)'); legend('PID', 'SMC', 'Switch');
xlim([0 time(end)]);
dist_pid = trapz(time, x_data_pid(:,1)); dist_smc = trapz(time, x_data_smc(:,1)); dist_sw = trapz(time, x_data_sw(:,1));
txt = sprintf('Distance traveled: PID - %2.2f, SMC - %2.2f, SW - %2.2f', dist_pid, dist_smc, dist_sw);
%annotation('textbox',[0.3 0.9 .1 .1],'String', txt, 'EdgeColor','k')
%%
figure(2);
plot(time, x_data_pid(:,2), time, x_data_smc(:,2), time, x_data_sw(:,2));
%patch([0 time(end) time(end) 0], [-5*pi/180, -5*pi/180, 5*pi/180, 5*pi/180], 'green', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);
yline(5*pi/180,'--','SMC', 'LabelVerticalAlignment', 'top' );
yline(5*pi/180,'--','PID', 'LabelVerticalAlignment', 'bottom', 'Alpha', 0 );
yline(-5*pi/180,'--','SMC', 'LabelVerticalAlignment', 'bottom' );
xlabel('Time (sec)'); ylabel('\theta (rad)'); legend('PID', 'SMC', 'Switch');
xlim([0 time(end)]);
%%
figure(3);
plot(time, u_data_pid(:,5), time, u_data_smc(:,5), time, u_data_sw(:,7));
xlabel('Time (sec)'); ylabel('Input variable, u (Volts)'); legend('PID', 'SMC', 'Switch');
xlim([0 time(end)]);
e_pid = trapz(time, abs(u_data_pid(:,1))); e_smc = trapz(time, abs(u_data_smc(:,1))); e_sw = trapz(time, abs(u_data_sw(:,5)));
txt = sprintf('Energy consumed: PID - %2.2f, SMC - %2.2f, SW - %2.2f', e_pid, e_smc, e_sw);
%annotation('textbox',[0.3 0.9 .1 .1],'String', txt, 'EdgeColor','k')