%% Plots_Guidance_and_Navigation_Project_Case_3

%% Errors
%% e_X,e_Y,e_Z-Time Diagram
figure('Name','e_X,e_Y,e_Z-Time Diagram')
subplot(3,1,1);
plot(e_X,'Linewidth',1.5)
title('Position Tracking Errors (Case3)')
xlabel('Time (sec)')
ylabel('e_X (m)')

subplot(3,1,2);
plot(e_Y,'Linewidth',1.5)
xlabel('Time (sec)')
ylabel('e_Y (m)')

subplot(3,1,3);
plot(e_Z,'Linewidth',1.5)
xlabel('Time (sec)')
ylabel('e_Z (m)')
%% e_Vg,e_gamma,e_zita-Time Diagram
figure('Name','e_Vg,e_gamma,e_zita-Time Diagram')
subplot(3,1,1);
plot(e_Vg,'Linewidth',1.5)
title('Speed(Vg),Elevatio(gamma),and Heading(zita) Tracking Errors(Case3)')
xlabel('Time (sec)')
ylabel('e_Vg (m/s)')

subplot(3,1,2);
plot(e_gammadeg,'Linewidth',1.5)
xlabel('Time (sec)')
ylabel('e_gamma (deg)')

subplot(3,1,3);
plot(e_zitadeg,'Linewidth',1.5)
xlabel('Time (sec)')
ylabel('e_zita (deg)')

%% UAV_and_Target_Flight_Path
figure('Name','Flight Path Diagram 3D')
plot3(X_UAV.Data,Y_UAV.Data,Z_UAV.Data,'Linewidth',1.5)
hold on
plot3(X_t.Data,Y_t.Data,Z_t.Data,'r--','Linewidth',1.5)
title('UAV and Target Flight Path 3D(Case3)')
legend('UAV','Target')
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')