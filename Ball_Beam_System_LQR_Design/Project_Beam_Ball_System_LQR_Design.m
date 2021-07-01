%% Project#1_Advanced_Control_Designing_LQR_Controller_For_Inverted_Pendulum_System_B10_21
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

Q = 5*eye(4);
R = 5;

%% Designing Controller Gain, LQR Method
M = [B_Linear A_Linear*B_Linear A_Linear^2*B_Linear A_Linear^3*B_Linear];
r_M = rank(M);
if r_M == min(size(M))
    fprintf('The system is controllable and the rank of M is\n')
    disp(r_M)
    
    [K_reg,P,E]=lqr(A_Linear,B_Linear,Q,R)

    fprintf('The Controller Gain "K" is\n')
    disp(K_reg)
else 
    disp('The System is Unctrollable')
end

%% Simulation
T = 200;
dt = 0.01;
X0 = [0.2;0;0.174533;0];
t = 0;
X(:,1) = X0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);
    u = -K_reg*Xj;
    D1 = Beam_Ball_LQR(t,Xj,u);
    D2 = Beam_Ball_LQR(t+dt/2,Xj+D1*dt/2,u);
    D3 = Beam_Ball_LQR(t+dt/2,Xj+D2*dt/2,u);
    D4 = Beam_Ball_LQR(t+dt,Xj+D3*dt,u);
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    Time(k+1) = t + dt;
    k = k + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:));
title('LQR Controller Design Initial Conddition "0.2,0,10,0" and R equals to "5"')
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