function [x2_dot] = plant(t, x, v)
global phi_0 d_1 d_2 d_3 d_4 d_5 d_6 c_0 c_1 c_2 c_3 theta_eq

alpha = @(t) sin(t) + 0.5;
beta = @(t) 40 * cos(t);

phi = pi/2 - x(1) - theta_eq + v;
phi_dot = -x(2) + v;
tau_ke = (c_2*phi^2+c_1*phi+c_0)*(1+c_3*phi_dot)*x(3) + v;
tau_p = d_1*(phi-phi_0) + d_2*phi_dot + d_3*exp(d_4*phi) - d_5*exp(d_6*phi) + v;

% x1_dot = x(2) + v;
x2_dot = alpha(t)*(tau_ke + tau_p) - beta(t)*(sin(x(1)-theta_eq)) + v;
% x3_dot = (u - x(3))/T_a;
end