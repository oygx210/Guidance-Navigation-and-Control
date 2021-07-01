%% T_Variable_Part_C_Guidance_and_Navigation_HW#3
clc
clear all
T = 0;
i = 1;
for T = 0:0.01:1
    sim('Problem_1_Part_C')
    MD1(i) = MD(end);
    CE1(i) = CE(end);
    i = i+1;
end

%% Miss_Distance
T = 0:0.01:1;
figure('Name','MD-T Diagram')
plot(T,MD1)
grid on
title('Miss Distance Variation vs Time Constant Variation')
xlabel('Guidance Time Constant (sec)')
ylabel('MD (m)')

%% Control_Effort
figure('Name','CE-Time Diagram')
plot(T,CE1)
grid on
title('Control Effort Variation vs Time Constant Variation')
xlabel('Guidance Time Constant (sec)')
ylabel('CE (m/s)')
