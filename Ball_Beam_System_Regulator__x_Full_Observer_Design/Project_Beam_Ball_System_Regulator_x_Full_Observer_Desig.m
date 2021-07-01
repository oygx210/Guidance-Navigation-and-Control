%% %% Project#1_Advanced_Control_Beam_Ball_System_Regulator_x_Full_Observer_Design
clc 
clear 

global m g J_Beam C_Yx Ko_Yx
%% Designing Controller Gain, Ackerman Method
m = 0.5; %% Disk Mass
g = 9.81;
J_Beam = 2;

A_Linear = [0 1 0 0;0 0 (-g)/(1.4) 0;0 0 0 1;(-m*g)/J_Beam 0 0 0];
B_Linear = [0;0;0;1/J_Beam];
C_Yx = [1 0 0 0];


M = [B_Linear A_Linear*B_Linear A_Linear^2*B_Linear A_Linear^3*B_Linear];
r_M = rank(M);
if r_M == min(size(M))
    fprintf('The system is controllable and the rank of M is\n')
    disp(r_M)
    
    mu_d = [-3 -4 -2+1i -2-1i]; %% Desired Eigenvalues
    K_Reg = acker(A_Linear,B_Linear,mu_d);

    fprintf('The Controller Gain "K" is\n')
    disp(K_Reg)
else 
    disp('The System is Unctrollable')
end

%% Designing Observer Gain When System Observes X1=x, Ackerman Method
N_T_Yx = [C_Yx' A_Linear'*C_Yx' (A_Linear')^2*C_Yx' (A_Linear')^3*C_Yx']; %% N tranpose matrix when system observs X1=x
r_N_T_Yx = rank(N_T_Yx);
if r_N_T_Yx == min(size(N_T_Yx))
    fprintf('When system observes (x), it is Observable and the rank of N_Transpose is\n')
    disp(r_N_T_Yx)
    
    muo_d = [-5 -5 -5 -5];
    Ko_T_Yx = acker(A_Linear',C_Yx',muo_d);
    Ko_Yx = Ko_T_Yx';

    fprintf('The Observer Gain "Ko" When System Observes (x) is\n')
    disp(Ko_Yx)
else 
    disp('The system is unobservable when it observes (x)')
end

%% Simulation
T = 30;
dt = 0.001;
X0 = [0.2;0;0.174533;0];
Xh0 = [0.01;0.01;0.01;0.01];
t = 0;
X(:,1) = X0;
Xh(:,1) = Xh0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);
    Xhj = Xh(:,k);
    y = C_Yx*Xj;
    u = -K_Reg*Xhj;
    D1 = Beam_Ball_Regul_FO_Proj(t,Xj,u);
    D2 = Beam_Ball_Regul_FO_Proj(t+dt/2,Xj+D1*dt/2,u);
    D3 = Beam_Ball_Regul_FO_Proj(t+dt/2,Xj+D2*dt/2,u);
    D4 = Beam_Ball_Regul_FO_Proj(t+dt,Xj+D3*dt,u);   
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    O1 = Beam_Ball_x_Full_Obser_Regual_Proj(t,Xhj,u,y);
    O2 = Beam_Ball_x_Full_Obser_Regual_Proj(t+dt/2,Xhj+O1*dt/2,u,y);
    O3 = Beam_Ball_x_Full_Obser_Regual_Proj(t+dt/2,Xhj+O2*dt/2,u,y);
    O4 = Beam_Ball_x_Full_Obser_Regual_Proj(t+dt,Xhj+O3*dt,u,y);   
    Xhj = Xhj + (O1+2*O2+2*O3+O4)/6*dt;
    Xh(:,k+1) = Xhj;
    Time(k+1) = t + dt;
    k = k + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:),Time,Xh(1,:),'g');
title('Regulator With Full Order Observer "x" Controller Design Initial Conddition "0.2,0,10,0"')
xlabel('time(s)')
ylabel('x(m)')
legend('X','Xhat','location','northeast')

subplot(4,1,2);plot(Time,X(2,:),Time,Xh(1,:),'g');
xlabel('time(s)')
ylabel('xdot(m/s)')
legend('X','Xhat','location','northeast')

subplot(4,1,3);plot(Time,X(3,:),Time,Xh(1,:),'g');
xlabel('time(s)')
ylabel('phi(rad)')
legend('X','Xhat','location','northeast')

subplot(4,1,4);plot(Time,X(4,:),Time,Xh(1,:),'g');
xlabel('time(s)')
ylabel('phidot(rad/s)')
legend('X','Xhat','location','northeast')
xlabel('time(s)')
ylabel('phidot(rad/s)')