% state variables
C_N0 = 0;
F_0 = 0;

% model parameters
tau_c = 20; % in ms; time constnat controling rise and decay of C_N
tau_1 = 50; % in ms; time constant of force decline at absence of strongly bound cross-bridges
tau_2 = 84.27; % in ms; time constant of force decline due to extra friction b/w actin and myosin from presence of strongly bound cross-bridges
tau_fat = 139.74*1000; % in s; time constant characterising the rate of recovery
A = 3; % in N ms^-1; scaling factor for forceand shortening velocity

A_rest = 1.44; % in N ms^-1
K_mrest = 0.27; 
tau_1rest = 25.92; % in ms

alpha_A = -4.15e-7; % in s^-2
alpha_K_m = 3.09e-8; % in N^-1 s^-1
alpha_tau_1 = 2.00e-5; % in N^-1

% R_0; % magnitude of enhancement in C_N form the following stimuli
K_m = 0.1; % sensitivity of strongly bound cross-bridges to C_N