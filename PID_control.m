% Created by: Haotao Ling
% Last updated: 06.10.2018

%%
close all;
clear;

delta_t = 0.01;
total_time = 5;
l_leg = 3;
desired_angle1 = deg2rad(40);
desired_angle2 = deg2rad(50);

theta_0 = deg2rad(10); %rad
theta_dot_0 = 0; %rad/s
a_ke_0 = 0;

alpha = 1.29; %1/(kg*m^2)
beta = 40.3; %N/(kg*m^2)
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

A = [0 1 0; (-beta*cos(theta_eq)-alpha*(d_1+d_3*d_4*exp(d_4*(pi/2-theta_eq))-d_5*d_6*exp(d_6*(pi/2-theta_eq)))) (-alpha*d_2) (alpha*(c_2*(pi/2-theta_eq)^2+c_1*(pi/2-theta_eq)+c_0)); 0 0 -1/T_a];
eig(A);

x_1 = theta_0;
x_2 = theta_dot_0;
x_3 = a_ke_0;
u = 0;

kp = 50; %10
kd = 10; %5
ki = 20; %10

x1s = [];
pes = [];
ts = 0:delta_t:total_time;

%x1_d = desired_angle1.* ones(length(0:delta_t:total_time), 1);
x1_d = deg2rad(10*sin(ts*2)+30);
%x1_d = [desired_angle1.* ones(ceil(length(0:delta_t:total_time)/2), 1); desired_angle2.* ones(floor(length(0:delta_t:total_time)/2), 1)];
x2_d = diff(x1_d);

j = 1;

for i = ts(1:end-1)
   pe = x1_d(j) - x_1;
   de = x2_d(j) - x_2;
   ie = sum(pe);
   u = pe*kp + de*kd + ki*ie;
   if u < 0
       u = 0;
   elseif u > 1
       u = 1;
   end
   
   x1s = [x1s x_1];
   pes = [pes pe];
   
   phi = pi/2 - x_1 - theta_eq;
   phi_dot = -x_2;
   tau_ke = (c_2*phi^2+c_1*phi+c_0)*(1+c_3*phi_dot)*x_3;
   tau_p = d_1*(phi-phi_0) + d_2*phi_dot + d_3*exp(d_4*phi) - d_5*exp(d_6*phi);
    
   x1_dot = x_2;
   x2_dot = alpha*(tau_ke + tau_p) - beta*(sin(x_1+theta_eq));
   x3_dot = (u - x_3)/T_a;
   
   x_1 = x_1 + x1_dot*delta_t;
   x_2 = x_2 + x2_dot*delta_t;
   x_3 = x_3 + x3_dot*delta_t;
   
   ankle_x = sin(pi-x_1)*l_leg;
   ankle_y = cos(pi-x_1)*l_leg;
   scatter(0, 0, 'filled');
   line([-l_leg, 0, ankle_x], [0, 0, ankle_y]); 
   axis([-5 5 -5 5])
   pause(delta_t)
   
   j = j+1;
end

figure;
subplot(2,1,1);
hold on;
plot(ts(1:end-1), rad2deg(x1s));
plot(ts, rad2deg(x1_d));
grid on
subplot(2,1,2);
plot(ts(1:end-1), rad2deg(pes));
grid on