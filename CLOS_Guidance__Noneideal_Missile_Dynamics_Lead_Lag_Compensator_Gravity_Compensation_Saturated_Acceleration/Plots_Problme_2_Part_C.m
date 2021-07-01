%% Plots_Guidance_and_Navigation_HW#2_Problem_2_Part_C

%% Missile
%% X_m-Time Diagram
figure('Name','X_m-Time Diagram')
plot(X_m)
title('Missile X Position Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('X_m (m)')

%% Y_m-Time Diagram
figure('Name','Y_m-Time Diagram')
plot(Y_m)
title('Missile Y Position Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('Y_m (m)')

%% Z_m-Time Diagram
figure('Name','Z_m-Time Diagram')
plot(Z_m)
title('Missile Z Position Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('Z_m (m)')

%% am_x-Time Diagram
figure('Name','am_x-Time Diagram')
plot(am_x)
title('Missile X Guidance Acceleration Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('am_x (m/s^2)')

%% am_y-Time Diagram
figure('Name','am_y-Time Diagram')
plot(am_y)
title('Missile Y Guidance Acceleration Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('am_y (m/s^2)')

%% am_z-Time Diagram
figure('Name','am_z-Time Diagram')
plot(am_z)
title('Missile Z Guidance Acceleration Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('am_z (m/s^2)')

%% Missile Flight Path Diagram 
figure('Name','Flight Path XY Plane Diagram')
plot(X_m.Data,Y_m.Data)
title('Missile Flight Path XY Plane(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('X (m)')
ylabel('Y (m)')

figure('Name','Flight Path XZ Plane Diagram')
plot(X_m.Data,Z_m.Data)
title('Missile Flight Path XZ Plane(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('X (m)')
ylabel('Z (m)')

figure('Name','Flight Path Diagram 3D')
plot3(X_m.Data,Y_m.Data,Z_m.Data)
title('Missile Flight Path 3D(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

%% Target
%% X_t-Time Diagram
figure('Name','X_t-Time Diagram')
plot(X_t)
title('Target X Position Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('X_t (m)')

%% Y_t-Time Diagram
figure('Name','Y_t-Time Diagram')
plot(Y_t)
title('Target Y Position Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('Y_t (m)')

%% Z_t-Time Diagram
figure('Name','Z_t-Time Diagram')
plot(Z_t)
title('Target Z Position Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('Z_t (m)')

%% Target Flight Path Diagram 
figure('Name','Flight Path XY Plane Diagram')
plot(X_t.Data,Y_t.Data)
title('Target Flight Path XY Plane(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('X (m)')
ylabel('Y (m)')

figure('Name','Flight Path XZ Plane Diagram')
plot(X_t.Data,Z_t.Data)
title('Target Flight Path XZ Plane(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('X (m)')
ylabel('Z (m)')

figure('Name','Flight Path Diagram 3D')
plot3(X_t.Data,Y_t.Data,Z_t.Data)
title('Target Flight Path 3D(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

%% Missile_and_Target_Flight_Path
figure('Name','Flight Path XY Plane Diagram')
plot(X_m.Data,Y_m.Data)
hold on
plot(X_t.Data,Y_t.Data)
title('Missile and Target Flight Path XY Plane(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
legend('Missile','Target')
xlabel('X (m)')
ylabel('Y (m)')

figure('Name','Flight Path XZ Plane Diagram')
plot(X_m.Data,Z_m.Data)
hold on
plot(X_t.Data,Z_t.Data)
title('Missile and Target Flight Path XZ Plane(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
legend('Missile','Target')
xlabel('X (m)')
ylabel('Z (m)')

figure('Name','Flight Path Diagram 3D')
plot3(X_m.Data,Y_m.Data,Z_m.Data)
hold on
plot3(X_t.Data,Y_t.Data,Z_t.Data)
title('Missile and Target Flight Path 3D(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
legend('Missile','Target')
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

%% Miss_Distance
figure('Name','MD-Time Diagram')
plot(MD)
title('Miss Distance Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('MD (m)')

%% Control_Effort
figure('Name','CE-Time Diagram')
plot(CE)
title('Control Effort Variation vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('CE (m/s)')

%% Missile_Distance_To_Elevation_LOS
figure('Name','d_epsilon-Time Diagram')
plot(d_epsilon)
title('Missile Distance To Elevation LOS vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('depsilon (m)')

%% Missile_Distance_To_Azimuth_LOS
figure('Name','d_sigma-Time Diagram')
plot(d_sigma)
title('Missile Distance To Azimuth LOS vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('dsigma (m)')

%% Missile_Distance_To_Elevation_LOS vs Missile_Distance_To_Azimuth_LOS 
figure('Name','Missile Distance To Elevation LOS vs Missile Distance To Azimuth LOS Diagram')
plot(d_sigma.Data,d_epsilon.Data)
title('Missile Distance To Elevation LOS vs Missile Distance To Azimuth LOS(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('dsigma (m)')
ylabel('depsilon (m)')

%% Difference_of_Missile_Elevation_LOS_Angle_and_Target_Elevation_LOS_Angle
figure('Name','delta_epsilon-Time Diagram')
plot(delta_epsilon)
title('Difference Of Missile Elevation LOS Angle & Target Elevation LOS Angle vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('delta epsilon (rad)')

%% Difference_of_Missile_Azimuth_LOS_Angle_and_Target_Azimuth_LOS_Angle
figure('Name','delta_sigma-Time Diagram')
plot(delta_sigma)
title('Difference Of Missile Azmiuth LOS Angle & Target Azimuth LOS Angle vs Time(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
xlabel('Time (sec)')
ylabel('delta sigma (rad)')

%% Acceleration_Command_VS_Actual_Acceleration_Azimuth_Channel 
figure('Name','Acceleration_Command_VS_Actual_Acceleration_Azimuth_Channel')
plot(ac_y)
hold on
plot(am_y)
title('Acceleration Command vs Actual Acceleration Azimuth Channel(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
legend('ac_y','am_y')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')

%% Acceleration_Command_VS_Actual_Acceleration_Elevation_Channel 
figure('Name','Acceleration_Command_VS_Actual_Acceleration_Elevation_Channel')
plot(ac_z)
hold on
plot(am_z)
title('Acceleration Command vs Actual Acceleration Elevation Channel(None Zero Dyanmic & Lead-Lag Guidance Coeff.)')
legend('ac_z','am_z')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')