%% emax_Variable_Part_F_Guidance_and_Navigation_HW#3
clc
clear all
e_max = 5;
i = 1;
for e_max = 5:1:20
    sim('Problem_1_Part_F')
    MD1(i) = MD(end);
    i = i+1;
end

%% Miss_Distance
e_max = 5:1:20;
figure('Name','MD-emax Diagram')
plot(e_max,MD1)
grid on
title('Miss Distance Variation vs Maximum Look Angle')
xlabel('emax (deg)')
ylabel('MD (m)')
