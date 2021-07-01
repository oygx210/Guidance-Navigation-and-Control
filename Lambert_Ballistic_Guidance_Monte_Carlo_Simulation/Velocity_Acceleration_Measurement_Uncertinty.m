%% Velocity_Measurement_Uncertinty_Guidance_and_Navigation_HW#4_Problem1_PartC1
clc
clear all
mu1 = 0;
sigma1 = 0.1;
Vu = normrnd(mu1,sigma1);
mu2 = 0;
sigma2 = 0.001;
au = normrnd(mu2,sigma2);
i = 1;
Xreal = 4095873;
for i = 1:1:100
    
    sim('Problem_1_Part_C3')
    mu1 = 0;
    sigma1 = 0.1;
    Vu = normrnd(mu1,sigma1);
    mu2 = 0;
    sigma2 = 0.001;
    au = normrnd(mu2,sigma2);
    Range(i) = X(end);
    
end

%% Range Error Standard Deviation and Plot
ERange = Xreal - Range;
i = 1:1:100;
figure('Name','Range Error_(Vu_au) Diagram')
plot(i,ERange)
grid on
title('Range Error vs Velocity and Acceleration Uncertainty')
xlabel('Number of Iterations')
ylabel('Range (m)')

Mean = mean(ERange);
Standard_Deviation = std(ERange);
%% Display
fprintf('The Range Error Average is (m)\n')
disp(Mean)

fprintf('The Range Error Standard Deviation is (m)\n')
disp(Standard_Deviation)