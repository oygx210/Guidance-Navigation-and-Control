%% Plots_Guidance_and_Navigation_HW#4_Problem_1_Part_B

%% Missile
%% X_m-Time Diagram
figure('Name','X-Time Diagram')
plot(t,X,'b')
grid on
title('Missile X Position Variation due to Time')
xlabel('Time (sec)')
ylabel('X (m)')

%% Y-Time Diagram
figure('Name','Y-Time Diagram')
plot(t,Y,'b')
grid on
title('Missile Y Position Variation due to Time')
xlabel('Time (sec)')
ylabel('Y (m)')

%% R-Time Diagram
figure('Name','R-Time Diagram')
plot(t,R,'b')
grid on
title('Missile Altitude Variation due to Time')
xlabel('Time (sec)')
ylabel('R (m)')

%% Longtitude-Time Diagram
figure('Name','Longtitude-Time Diagram')
plot(t,Theta,'b')
grid on
title('Longtitude Variation due to Time')
xlabel('Time (sec)')
ylabel('Longtitude (deg)')

%% Velocity-Time Diagram
figure('Name','Velocity-Time Diagram')
plot(t,V,'b')
grid on
title('Missile Velocity Variation due to Time')
xlabel('Time (sec)')
ylabel('V (m/s)')

%% Total R-Acceleration-Time Diagram
figure('Name','Total R-Acceleration-Time Diagram')
plot(t,atotal_R,'b')
grid on
title('Total R-Acceleration due to Time')
xlabel('Time (sec)')
ylabel('atotal_R (m/s^2)')

%% Total Theta-Acceleration-Time Diagram
figure('Name','Total Theta-Acceleration-Time Diagram')
plot(t,atotal_Theta,'b')
grid on
title('Total Theta-Acceleration due to Time')
xlabel('Time (sec)')
ylabel('atotal_Theta (m/s^2)')

%% Required and Instantenous X-Velocity-Time Diagram
figure('Name','Required and Instantenous X-Velocity-Time Diagram')
plot(t,V_X,'b')
grid on
hold on
plot(t,Vr_X,'g')
grid on
legend('Vx','Vrx')
title('Required and Instantenous X-Velocity due to Time')
xlabel('Time (sec)')
ylabel('X-Vlocity (m/s)')
axis([0 48.794 -inf inf])
%% Required and Instantenous Y-Velocity-Time Diagram
figure('Name','Required and Instantenous Y-Velocity-Time Diagram')
plot(t,V_X,'b')
grid on
hold on
plot(t,Vr_X,'g')
grid on
legend('Vy','Vry')
title('Required and Instantenous Y-Velocity due to Time')
xlabel('Time (sec)')
ylabel('Y-Vlocity (m/s)')
axis([0 48.794 -inf inf])