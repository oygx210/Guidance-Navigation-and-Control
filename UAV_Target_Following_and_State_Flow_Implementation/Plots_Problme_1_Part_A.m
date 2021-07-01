%% Plots_Guidance_and_Navigation_HW#5_Problem_1_Part_A

%% X_UAV-Time Diagram
figure('Name','X_UAV-Time Diagram')
plot(X_UAV)
grid on
title('UAV X Position Variation due to Time')
xlabel('Time (sec)')
ylabel('X (m)')

%% Y_UAV-Time Diagram
figure('Name','Y_UAV-Time Diagram')
plot(Y_UAV)
grid on
title('UAV Y Position Variation due to Time')
xlabel('Time (sec)')
ylabel('Y (m)')

%% UAV Flight Path Diagram
figure('Name','Flight Path XY Plane Diagram')
plot(X_UAV.Data,Y_UAV.Data)
grid on
title('UAV Flight Path XY Plane')
xlabel('X (m)')
ylabel('Y (m)')

%% Target Movement Path
pgon = nsidedpoly(6,'Center',[750 433],'SideLength',500);
figure('Name','Target Movement Path')
plot(pgon)
axis equal
title('Target Movement Path Variation due to Time')
xlabel('X (m)')
ylabel('Y (m)')

%% Xdot_UAV-Time Diagram
figure('Name','Xdot_UAV-Time Diagram')
plot(Xdot_UAV)
grid on
title('UAV X Velocity Variation due to Time')
xlabel('Time (sec)')
ylabel('Xdot (m/s)')

%% Ydot_UAV-Time Diagram
figure('Name','Ydot_UAV-Time Diagram')
plot(Ydot_UAV)
grid on
title('UAV Y Velocity Variation due to Time')
xlabel('Time (sec)')
ylabel('Ydot (m/s)')

%% Velocity-Time Diagram
figure('Name','Velocity-Time Diagram')
plot(V_UAV)
grid on
title('UAV Velocity Variation due to Time')
xlabel('Time (sec)')
ylabel('V (m/s)')

%%  Psi-Time Diagram
figure('Name','Psi-Time Diagram')
plot(Psi_UAV)
grid on
title('UAV Heading Angle Variation due to Time')
xlabel('Time (sec)')
ylabel('Psi (deg)')

%% Target Movemment-Time Diagram
figure('Name','Target Movemment-Time Diagram')
plot(Target_Movement)
grid on
title('Target Movement Variation due to Time')
xlabel('Time (sec)')
ylabel('Position Number')

%% Distance Between UAV and Target-Time Diagram
figure('Name','Distance Between UAV and Target-Time Diagram')
plot(d)
grid on
title('Distance Between UAV and Target Variation due to Time')
xlabel('Time (sec)')
ylabel('d (m)')

%% Commanded X Acceleration-Time Diagram
figure('Name','Commanded X Acceleration-Time Diagram')
plot(ax_UAV)
grid on
title('Commanded X Acceleration Variation due to Time')
xlabel('Time (sec)')
ylabel('ax (m/(s^2))')

%% Commanded Y Acceleration-Time Diagram
figure('Name','Commanded Y Acceleration-Time Diagram')
plot(ay_UAV)
grid on
title('Commanded Y Acceleration Variation due to Time')
xlabel('Time (sec)')
ylabel('ay (m/(s^2))')
