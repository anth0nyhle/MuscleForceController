function [x2_dot_hat] = big_F(t, x)
global phi_0 d_1 d_2 d_3 d_4 d_5 d_6 c_0 c_1 c_2 c_3 theta_eq

alpha_hat = @(t) 2;
beta_hat = @(t) 44;

phi = pi/2 - x(1) - theta_eq;
phi_dot = -x(2);
tau_ke = (c_2*phi^2+c_1*phi+c_0)*(1+c_3*phi_dot)*x(3);
tau_p = d_1*(phi-phi_0) + d_2*phi_dot + d_3*exp(d_4*phi) - d_5*exp(d_6*phi);

% x1_dot_hat = x(2);
x2_dot_hat = alpha_hat(t)*(tau_ke + tau_p) - beta_hat(t)*(sin(x(1)-theta_eq));
% x3_dot = (u - x(3))/T_a;
end