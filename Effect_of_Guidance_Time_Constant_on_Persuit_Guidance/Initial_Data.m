%% Initial_Data_Guidance_and_Navigation_HW#3

ro = 1.225; %kg/m^3%
S = 0.1; %m^2%
g = 0; %m/s^2%
aty = 0; %m/s^2%d
atz = 0; %m/s^2%
G_p = 7.5;
T_1 = 0;
T_2 = 0;
R_min = 10;

Psi_m0 = 10*pi/180;
Theta_m0 = 24*pi/180;
V_m0B = [1000;0;0];
R_Psi = [cos(Psi_m0) sin(Psi_m0) 0;-sin(Psi_m0) cos(Psi_m0) 0;0 0 1];
R_Theta = [cos(Theta_m0) 0 -sin(Theta_m0);0 1 0;sin(Theta_m0) 0 cos(Theta_m0)];
R_BI = R_Theta*R_Psi;
R_IB = R_BI';
V_m0I = V_m0B;