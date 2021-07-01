%% Rmin_Variable_Part_E_Guidance_and_Navigation_HW#3
clc
clear all
R_min = 100;
i = 1;
for R_min = 100:10:2000
    sim('Problem_1_Part_E')
    MD1(i) = MD(end);
    i = i+1;
end

%% Miss_Distance
R_min = 100:10:2000;
figure('Name','MD-Rmin Diagram')
plot(R_min,MD1)
grid on
title('Miss Distance Variation vs Minimum Allowable Distance Between Missile and Target')
xlabel('Rmin (m)')
ylabel('MD (m)')
