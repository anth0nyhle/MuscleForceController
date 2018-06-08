% main
close all;
clear;

% global tau_c tau_1 tau_2 A;
global tau_c tau_2 tau_fat A_rest K_mrest tau_1rest alpha_A alpha_K_m alpha_tau_1;
parameters;

IPI = input('Interphase Interval (IPI; ms): ');
% CFT120 = 8.3hz;
% CFT100 = 10hz;
% CFT70 = 14.3hz
% CFT50 = 20hz; 
% CFT40 = 25hz; 
% CFT33 = 30hz; 
% CFT25 = 40hz;
% CFT20 = 50hz; 
% CFT10 = 60hz;

stim_t = input('Stimulation duration (ms): '); % in ms

t_span = [0 2000];
int = [C_N0; F_0; A_rest; K_mrest; tau_1rest];

options = [];

[t, dY] = ode45(@forcefatigue_ode, t_span, int, options, IPI, stim_t); % fatigue model
% output col vec for t
% output 2 col matrix for dY, 1st col: C_N, 2nd col: F

% Model2_CFT100 = [t(:, 1), dY(:, 2)];

figure(1);
% subplot(2, 1, 1);
% plot(t, dY(:, 1));
% xlabel('Time (s)');
% ylabel('C_{N}');
% % xlim([0 300]);
% ylim([0 0.1+max(dY(:, 1))]);
% subplot(2, 1, 2);
plot(t/1000, dY(:, 2));
xlabel('Time (s)');
ylabel('Force (N)');
% xlim([0 300]);
ylim([0 100+max(dY(:, 2))]);
% suptitle(['IPI = ' num2str(IPI) ', ' num2str(stim_t) ' pulses'])

figure(2);
subplot(3, 1, 1);
plot(t, dY(:, 3));
xlabel('Time (s)');
ylabel('A');

subplot(3, 1, 2);
plot(t, dY(:, 4));
xlabel('Time (s)');
ylabel('K_{m}');

subplot(3, 1, 3);
plot(t, dY(:, 5));
xlabel('Time (s)');
ylabel('tau_{1}');