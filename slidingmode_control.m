% Created by: Anthony Le
% Last updated: 06.09.2018

% ME 599: Nonlinear Dynamic Analysis
% Project
%% Sliding Mode Control
close all;
clear;

global phi_0 d_1 d_2 d_3 d_4 d_5 d_6 c_0 c_1 c_2 c_3 theta_eq T_a

delta_t = 0.01;
total_time = 3;
l_leg = 3;

theta_0 = deg2rad(60); %rad
theta_dot_0 = 0; %rad/s
a_ke_0 = 0;

% alpha = 1.29; %1/(kg*m^2)
% beta = 40.3; %N/(kg*m^2)
phi_0 = 2.81E-14; %rad
d_1 = 0.863; %N*m
d_2 = 2.56; %N*m
d_3 = 1E-10; %N*m
d_4 = 15.9;
d_5 = 0.405; %N*m
d_6 = -94.2;
c_0 = 50.9; %(N*m)
c_1 = 163; %(N*m)
c_2 = -125; %(N*m)
c_3 = 1.28;
theta_eq = 0.416; %rad
T_a = 0.18; %s

% x_1 = theta_0;
% x_2 = theta_dot_0;
x_3 = a_ke_0;
% u = 1;

% x = [x_1, x_2, x_3];

% phi = pi/2 - x(1) - theta_eq;
% phi_dot = -x(2);
% tau_ke = (c_2*phi^2+c_1*phi+c_0)*(1+c_3*phi_dot)*x(3);
% tau_p = d_1*(phi-phi_0) + d_2*phi_dot + d_3*exp(d_4*phi) - d_5*exp(d_6*phi);
% x1_dot = x(2);
% x2_dot = alpha(t)*(tau_ke + tau_p) - beta(t)*(sin(x(1)-theta_eq));
% x3_dot = (u - x(3))/T_a;

% alpha = @(t) sin(t) + 0.5;
% beta = @(t) 40 * cos(t);
% f = @(t, x) [phi; phi_dot; tau_ke; tau_p; x1_dot; x2_dot; x3_dot];
% plant = @(t, x, v) [phi + v; phi_dot + v; tau_ke + v; tau_p + v; x1_dot + v; x2_dot + v; x3_dot + v];

% phi_hat = pi/2 - x(1) - theta_eq;
% phi_dot_hat = -x(2);
% tau_ke_hat = (c_2*phi_hat^2+c_1*phi_hat+c_0)*(1+c_3*phi_dot_hat)*x(3);
% tau_p_hat = d_1*(phi_hat-phi_0) + d_2*phi_dot_hat + d_3*exp(d_4*phi_hat) - d_5*exp(d_6*phi_hat);
% x1_dot_hat = x(2);
% x2_dot_hat = alpha_hat(t)*(tau_ke_hat + tau_p_hat) - beta_hat(t)*(sin(x(1)-theta_eq));
% x3_dot_hat = (u - x(3))/T_a;

% alpha_hat = @(t) 1.3;
% beta_hat = @(t) 40;
% f_hat = @(t, x) [phi_hat; phi_dot_hat; tau_ke_hat; tau_p_hat; x1_dot_hat; x2_dot_hat; x3_dot_hat];

% alpha_limit = @(t) 2;
% beta_limit = @(t) 45;
% F =  @(t, x) [phi_hat; phi_dot_hat; tau_ke_hat; tau_p_hat; x1_dot_hat; x2_dot_hat; x3_dot_hat];

u_min = 0;
u_max = 1;

t_step = 0:delta_t:total_time;

% desired trajectory
pos_1 = 0.95;
pos_2 = 1.19;
desired_angle1 = deg2rad(40);
desired_angle2 = deg2rad(50);

% traj = [zeros(50, 1); repmat(pos_1, 50, 1); repmat(pos_2, 50, 1); repmat(pos_1, 50, 1);
%         repmat(pos_2, 50, 1); repmat(pos_1, 51, 1);];
traj = [desired_angle1.* ones(ceil(length(0:delta_t:total_time)/2), 1); desired_angle2.* ones(floor(length(0:delta_t:total_time)/2), 1)];
% traj = sin(pi * t_step')/9 + 1;
traj_dot = diff(traj)/delta_t; traj_dot = [traj_dot(1); traj_dot];
traj_ddot = diff(traj_dot)/delta_t; traj_ddot = [traj_ddot(1); traj_ddot];

TRAJ = [traj, traj_dot, traj_ddot];

x3 = x_3;

x = [TRAJ(1, 1), TRAJ(1, 2), x3];

% x1 = TRAJ(1, 1);
% x2 = TRAJ(1, 2);
% x3 = x_3;
u = 0;

% controller
lambda = 100;
eta = 0.1;

s_func = @(x_err) x_err(2)+lambda*x_err(1);

xs = -100*(ones(length(t_step),3));
vs = -100*(ones(length(t_step),1));
ss = -100*(ones(length(t_step),1));

for i = 1:length(t_step)   
    % determine control input
    x_err = x(1:2) - TRAJ(i, 1:2); % don't care about error in acceleration because that is an input!
    u_hat = -f_hat(t_step(i), x) + TRAJ(i, 3) - lambda * x_err(2);
    k = big_F(t_step(i), x) + eta;
    s = s_func(x_err);
    u = u_hat - k * sign(s);

    % run dynamics forward
    x1_dot = x(2);
    x2_dot = plant(t_step(i), x, u);
    x3_dot = (u - x(3)) / T_a;
    x(1) = x(1) + delta_t * x1_dot;
    x(2) = x(2) + delta_t * x2_dot;
    x(3) = x(3) + delta_t * x3_dot;
    
    xs(i, :) = x;
    vs(i) = u;
    ss(i) = s;
end

% plotting
% position
subplot(3,2,1)
hold on
plot(t_step, TRAJ(:,1), 'r-')
plot(t_step, xs(:,1), 'bx-')
title('Position')
legend('Ref', 'Actual')

% position error
subplot(3,2,3)
hold on
plot(t_step, xs(:,1) - TRAJ(:,1), 'b-')
title('Position Tracking Error')

% velocity
subplot(3,2,2)
hold on
plot(t_step, TRAJ(:,2), 'r-')
plot(t_step, xs(:,2), 'bx-')
title('Velocity')
legend('Ref', 'Actual')

% velocity error
subplot(3,2,4)
hold on
plot(t_step, xs(:,2) - TRAJ(:,2), 'k-')
title('Position Tracking Error')

% control input
subplot(3,2,5)
plot(t_step, vs, 'k-')
title('Control Input')

% s plot
subplot(3,2,6)
plot(t_step, ss, 'b-')
title('s plot')
% fprintf('Switch Count: %d \n', switch_count)
