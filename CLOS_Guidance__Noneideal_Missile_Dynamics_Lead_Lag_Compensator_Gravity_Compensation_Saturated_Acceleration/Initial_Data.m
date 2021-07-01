%% Initial_Data_Guidance_and_Navigation_HW#2
ro = 1.225; %kg/m^3%
S = 0.1; %m^2%
g = 9.81; %m/s^2%
aty = 0.5*g; %m/s^2%
atz = 0.5*g; %m/s^2%
Gz = 2; %Constant Guidance Coefficient Elevation Channel%
Gy = 2; %Constant Guidance Coefficient Azimuth Channel%
% A = 0.07;
% B = A*15;
gain = 9.07;
%% Transfer_Function_of_Lead-Lag_Compensator
num_lead_comp = [1 A];
den_lean_comp = [1 B];
lead_comp = tf(num_lead_comp,den_lean_comp);

num_controller = [0 1];
den_controller = [0.2 1];
controller = tf(num_controller,den_controller);

open_loop_tf = lead_comp*controller;
close_loop_tf = feedback(open_loop_tf,1);

%rlocus(open_loop_tf);