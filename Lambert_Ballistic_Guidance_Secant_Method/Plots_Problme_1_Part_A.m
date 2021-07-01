%% Plots_Guidance_and_Navigation_HW#4_Problem_1_Part_A

%% Missile
%% X_m-Time Diagram
figure('Name','X-Time Diagram')
plot(X)
grid on
title('Missile X Position Variation due to Time')
xlabel('Time (sec)')
ylabel('X (m)')

%% Y-Time Diagram
figure('Name','Y-Time Diagram')
plot(Y)
grid on
title('Missile Y Position Variation due to Time')
xlabel('Time (sec)')
ylabel('Y (m)')

%% R-Time Diagram
figure('Name','R-Time Diagram')
plot(R)
grid on
title('Missile Altitude Variation due to Time')
xlabel('Time (sec)')
ylabel('R (m)')

%% Longtitude-Time Diagram
figure('Name','Longtitude-Time Diagram')
plot(Theta)
grid on
title('Longtitude Variation due to Time')
xlabel('Time (sec)')
ylabel('Longtitude (deg)')

%% Gamma-Time Diagram
figure('Name','Gamma-Time Diagram')
plot(Gamma)
grid on
title('Missile Flight Path Angle Variation due to Time')
xlabel('Time (sec)')
ylabel('Gamma (deg)')

%% Velocity-Time Diagram
figure('Name','Velocity-Time Diagram')
plot(V)
grid on
title('Missile Velocity Variation due to Time')
xlabel('Time (sec)')
ylabel('V (m/s)')