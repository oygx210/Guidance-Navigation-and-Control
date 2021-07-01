%% Project#1_Advanced_Control_Ball_Beam_System_Regulator_Design
clc 
clear 

global m g J_Beam
%% System Parameters
m = 0.5; %% Disk Mass
g = 9.81;
J_Beam = 2;

A_Linear = [0 1 0 0;0 0 (-g)/(1.4) 0;0 0 0 1;(-m*g)/J_Beam 0 0 0];
B_Linear = [0;0;0;1/J_Beam];
C_Yx = [1 0 0 0];
C_Yphi = [0 0 1 0];

%% Designing Controller Gain, Ackerman Method
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

%% Designing Observer Gain When System Observes X3=theta, Ackerman Method
N_T_Yphi = [C_Yphi' A_Linear'*C_Yphi' (A_Linear')^2*C_Yphi' (A_Linear')^3*C_Yphi']; %% N tranpose matrix when system observs X3=theta
r_N_T_Yphi = rank(N_T_Yphi);
if r_N_T_Yphi == min(size(N_T_Yphi))
    fprintf('When system observes (phi), it is Observable and the rank of N_Transpose is\n')
    disp(r_N_T_Yphi)
    
    muo_d = [-5 -5 -5 -5];
    Ko_T_Yphi = acker(A_Linear',C_Yphi',muo_d);
    Ko_Yphi = Ko_T_Yphi';

    fprintf('The Observer Gain "Ko" When System Observes (phi) is\n')
    disp(Ko_Yphi)
else 
    disp('The system is unobservable when it observes (phi)')
end

%% Simulation
T = 5;
dt = 0.01;
X0 = [0.1;0;0.0872665;0];
t = 0;
X(:,1) = X0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);
    u = -K_Reg*Xj;
    D1 = Beam_Ball_Regul_Proj(t,Xj,u);
    D2 = Beam_Ball_Regul_Proj(t+dt/2,Xj+D1*dt/2,u);
    D3 = Beam_Ball_Regul_Proj(t+dt/2,Xj+D2*dt/2,u);
    D4 = Beam_Ball_Regul_Proj(t+dt,Xj+D3*dt,u);
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    Time(k+1) = t + dt;
    k = k+1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:));
title('Regulator Controller Design Initial Conddition "0.1,0,5,0"')
xlabel('time(s)')
ylabel('x(m)')

subplot(4,1,2);plot(Time,X(2,:));
xlabel('time(s)')
ylabel('xdot(m/s)')

subplot(4,1,3);plot(Time,X(3,:));
xlabel('time(s)')
ylabel('phi(rad)')

subplot(4,1,4);plot(Time,X(4,:));
xlabel('time(s)')
ylabel('phidot(rad/s)')