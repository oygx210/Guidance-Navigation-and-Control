%% delT_Variable_Part_G_Guidance_and_Navigation_HW#3
clc
clear all
delT = 0.01;
i = 1;
for delT = 0.01:0.01:0.2
    sim('Problem_1_Part_G')
    MD1(i) = MD(end);
    i = i+1;
end

%% Miss_Distance
delT = 0.01:0.01:0.2;
figure('Name','MD-delT Diagram')
plot(delT,MD1)
grid on
title('Miss Distance Variation vs Zero Order Hold Acceleration Command')
xlabel('delT (s)')
ylabel('MD (m)')
