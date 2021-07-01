%% alim_Variable_Part_D_Guidance_and_Navigation_HW#3
clc
clear all
alim = 10;
i = 1;
for alim = 10:1:100
    sim('Problem_1_Part_D')
    MD1(i) = MD(end);
    CE1(i) = CE(end);
    i = i+1;
end

%% Miss_Distance
alim = 10:1:100;
figure('Name','MD-alim Diagram')
plot(alim,MD1)
grid on
title('Miss Distance Variation vs Maximum Allowable Accelertion')
xlabel('Limited Acceleration (G)')
ylabel('MD (m)')

%% Control_Effort
figure('Name','CE-alim Diagram')
plot(alim,CE1)
grid on
title('Control Effort Variation vs Maximum Allowable Accelertion')
xlabel('Limited Acceleration (m/s^2)')
ylabel('CE (m/s)')
