%% Plots_Guidance_and_Navigation_HW#3_Problem_1_Part_A

%% Missile
%% X_m-Time Diagram
figure('Name','X_m-Time Diagram')
plot(X_m)
grid on
title('Missile X Position Variation vs Time')
xlabel('Time (sec)')
ylabel('X_m (m)')

%% Y_m-Time Diagram
figure('Name','Y_m-Time Diagram')
plot(Y_m)
grid on
title('Missile Y Position Variation vs Time')
xlabel('Time (sec)')
ylabel('Y_m (m)')

%% Z_m-Time Diagram
figure('Name','Z_m-Time Diagram')
plot(Z_m)
grid on
title('Missile Z Position Variation vs Time')
xlabel('Time (sec)')
ylabel('Z_m (m)')

%% am_x-Time Diagram
figure('Name','am_x-Time Diagram')
plot(am_x)
grid on
title('Missile X Guidance Acceleration Variation vs Time')
xlabel('Time (sec)')
ylabel('am_x (m/s^2)')

%% am_y-Time Diagram
figure('Name','am_y-Time Diagram')
plot(am_y)
grid on
title('Missile Y Guidance Acceleration Variation vs Time')
xlabel('Time (sec)')
ylabel('am_y (m/s^2)')

%% am_z-Time Diagram
figure('Name','am_z-Time Diagram')
plot(am_z)
grid on
title('Missile Z Guidance Acceleration Variation vs Time')
xlabel('Time (sec)')
ylabel('am_z (m/s^2)')

%% Missile Flight Path Diagram 
figure('Name','Flight Path XY Plane Diagram')
plot(X_m.Data,Y_m.Data)
grid on
title('Missile Flight Path XY Plane')
xlabel('X (m)')
ylabel('Y (m)')

figure('Name','Flight Path XZ Plane Diagram')
plot(X_m.Data,Z_m.Data)
grid on
title('Missile Flight Path XZ Plane')
xlabel('X (m)')
ylabel('Z (m)')

figure('Name','Flight Path Diagram 3D')
plot3(X_m.Data,Y_m.Data,Z_m.Data)
title('Missile Flight Path 3D')
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

%% Target
%% X_t-Time Diagram
figure('Name','X_t-Time Diagram')
plot(X_t)
grid on
title('Target X Position Variation vs Time')
xlabel('Time (sec)')
ylabel('X_t (m)')

%% Y_t-Time Diagram
figure('Name','Y_t-Time Diagram')
plot(Y_t)
grid on
title('Target Y Position Variation vs Time')
xlabel('Time (sec)')
ylabel('Y_t (m)')

%% Z_t-Time Diagram
figure('Name','Z_t-Time Diagram')
plot(Z_t)
grid on
title('Target Z Position Variation vs Time')
xlabel('Time (sec)')
ylabel('Z_t (m)')

%% Target Flight Path Diagram 
figure('Name','Flight Path XY Plane Diagram')
plot(X_t.Data,Y_t.Data)
grid on
title('Target Flight Path XY Plane')
xlabel('X (m)')
ylabel('Y (m)')

figure('Name','Flight Path XZ Plane Diagram')
plot(X_t.Data,Z_t.Data)
grid on
title('Target Flight Path XZ Plane')
xlabel('X (m)')
ylabel('Z (m)')

figure('Name','Flight Path Diagram 3D')
plot3(X_t.Data,Y_t.Data,Z_t.Data)
title('Target Flight Path 3D')
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

%% Missile_and_Target_Flight_Path
figure('Name','Flight Path XY Plane Diagram')
plot(X_m.Data,Y_m.Data)
grid on
hold on
plot(X_t.Data,Y_t.Data)
grid on
title('Missile and Target Flight Path XY Plane')
legend('Missile','Target')
xlabel('X (m)')
ylabel('Y (m)')

figure('Name','Flight Path XZ Plane Diagram')
plot(X_m.Data,Z_m.Data)
grid on
hold on
plot(X_t.Data,Z_t.Data)
grid on
title('Missile and Target Flight Path XZ Plane')
legend('Missile','Target')
xlabel('X (m)')
ylabel('Z (m)')

figure('Name','Flight Path Diagram 3D')
plot3(X_m.Data,Y_m.Data,Z_m.Data)
grid on
hold on
plot3(X_t.Data,Y_t.Data,Z_t.Data)
grid on
title('Missile and Target Flight Path 3D')
legend('Missile','Target')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

%% Miss_Distance
figure('Name','MD-Time Diagram')
plot(MD)
grid on
title('Miss Distance Variation vs Time')
xlabel('Time (sec)')
ylabel('MD (m)')

%% Control_Effort
figure('Name','CE-Time Diagram')
plot(CE)
grid on
title('Control Effort Variation vs Time')
xlabel('Time (sec)')
ylabel('CE (m/s)')

%% Acceleration_Command_VS_Actual_Acceleration_Azimuth_Channel 
figure('Name','Acceleration_Command_VS_Actual_Acceleration_Azimuth_Channel')
plot(ac_y)
grid on
hold on
plot(am_y)
grid on
title('Acceleration Command vs Actual Acceleration Azimuth Channel')
legend('ac_y','am_y')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')

%% Acceleration_Command_VS_Actual_Acceleration_Elevation_Channel 
figure('Name','Acceleration_Command_VS_Actual_Acceleration_Elevation_Channel')
plot(ac_z)
grid on
hold on
plot(am_z)
grid on
title('Acceleration Command vs Actual Acceleration Elevation Channel')
legend('ac_z','am_z')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')

%% Look_Angle
figure('Name','L-Time Diagram')
plot(L)
grid on
title('Look Angle Variation vs Time')
xlabel('Time (sec)')
ylabel('L (deg)')