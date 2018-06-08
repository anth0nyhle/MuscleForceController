% Created by: Anthony Le
% Last updated: 06.07.2018

% PID controller tutorial example
% https://www.ee.usyd.edu.au/tutorials_online/matlab/PID/PID.html
%%
num = 1;
den = [1 10 20];
% step(num, den);
Kp = 500;
Ki = 500;
Kd = 100;

numc = [Kd Kp Ki];
denc = [1 0];
[numCL, denCL] = cloop(conv(num, numc), conv(den, denc));
t = 0:0.01:2;
step(numCL, denCL, t);
% axis([0 100 0 1.5]);