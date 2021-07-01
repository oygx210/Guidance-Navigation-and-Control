%% Nonelinear_Control_Project_Plots

%% X-Time Diagram

figure('Name','X-Time Diagram')
plot(x)
title('XInertia variation due to Time')
xlabel('Time (sec)')
ylabel('X (m)')

%% Y-Time Diagram

figure('Name','Y-Time Diagram')
plot(y)
title('YInertia variation due to Time')
xlabel('Time (sec)')
ylabel('Y (m)')

%% Z-Time Diagram

figure('Name','Z-Time Diagram')
plot(z)
title('ZInertia variation due to Time')
xlabel('Time (sec)')
ylabel('z (m)')

%% U-Time Diagram

figure('Name','U-Time Diagram')
plot(U)
title('Uvelocity variation due to Time')
xlabel('Time (sec)')
ylabel('U (m/s)')

%% V-Time Diagram

figure('Name','V-Time Diagram')
plot(V)
title('Vvelocity variation due to Time')
xlabel('Time (sec)')
ylabel('V (m/s)')

%% W-Time Diagram

figure('Name','W-Time Diagram')
plot(W)
title('Wvelocity variation due to Time')
xlabel('Time (sec)')
ylabel('W (m/s)')

%% Roll velocity-Time Diagram

figure('Name','Roll velocity-Time Diagram')
plot(P)
title('Roll velocity variation due to Time')
xlabel('Time (sec)')
ylabel('P (rad/s)')

%% Pitch velocity-Time Diagram

figure('Name','Pitch velocity-Time Diagram')
plot(Q)
title('Pitch velocity variation due to Time')
xlabel('Time (sec)')
ylabel('Q (rad/s)')

%% Yaw velocity-Time Diagram

figure('Name','Yaw velocity-Time Diagram')
plot(R)
title('Yaw velocity variation due to Time')
xlabel('Time (sec)')
ylabel('R (rad/s)')

%% Phi-Time Diagram

figure('Name','Phi-Time Diagram')
plot(Phi)
title('Phi variation due to Time')
xlabel('Time (sec)')
ylabel('Phi (rad)')

%% Theta-Time Diagram

figure('Name','Theta-Time Diagram')
plot(Teta)
title('Theta variation due to Time')
xlabel('Time (sec)')
ylabel('Theta (rad)')

%% Psi-Time Diagram

figure('Name','Psi-Time Diagram')
plot(Psi)
title('Psi variation due to Time')
xlabel('Time (sec)')
ylabel('Psi (rad)')

%% U1-Time Diagram

figure('Name','U1-Time Diagram')
plot(u1)
title('U1 variation due to Time')
xlabel('Time (sec)')
ylabel('U1 (N)')

%% U2-Time Diagram

figure('Name','U2-Time Diagram')
plot(u2)
title('U2 variation due to Time')
xlabel('Time (sec)')
ylabel('U2 (N)')

%% U3-Time Diagram

figure('Name','U3-Time Diagram')
plot(u3)
title('U3 variation due to Time')
xlabel('Time (sec)')
ylabel('U3 (N)')
%% U4-Time Diagram

figure('Name','U4-Time Diagram')
plot(u4)
title('U4 variation due to Time')
xlabel('Time (sec)')
ylabel('U4 (N)')
%% dU1-Time Diagram

figure('Name','dU1-Time Diagram')
plot(u1dot)
title('dU1 variation due to Time')
xlabel('Time (sec)')
ylabel('dU1 (N/s)')

%% dU2-Time Diagram

figure('Name','dU2-Time Diagram')
plot(u2dot)
title('dU2 variation due to Time')
xlabel('Time (sec)')
ylabel('dU2 (N/s)')

%% dU3-Time Diagram

figure('Name','dU3-Time Diagram')
plot(u3dot)
title('dU3 variation due to Time')
xlabel('Time (sec)')
ylabel('dU3 (N/s)')
%% dU4-Time Diagram

figure('Name','dU4-Time Diagram')
plot(u4dot)
title('dU4 variation due to Time')
xlabel('Time (sec)')
ylabel('dU4 (N/s)')
%% S1-Time Diagram

figure('Name','S1-Time Diagram')
plot(s1)
title('S1 variation due to Time')
xlabel('Time (sec)')
ylabel('S1')
%% S3-Time Diagram

figure('Name','S3-Time Diagram')
plot(s3)
title('S3 variation due to Time')
xlabel('Time (sec)')
ylabel('S3')
%% Flight Path Diagram 

figure('Name','Flight Path Diagram')
plot3(x.Data,y.Data,z.Data)
title('Flight Path')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
