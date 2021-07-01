%% Project#1_Advanced_Control_Designing_LQR_Controller_For_Inverted_Pendulum_System_B10_21
clc 
clear
global M_Cart m g l

%% System Parameters
M_Cart = 2; %% Cart Mass
m = 0.5; %% Pendulum Mass
l = 1;  %% Pendulum Beam Length
g = 9.81;

A_Linear = [0 1 0 0;0 0 (-m*g)/(M_Cart) 0;0 0 0 1; 0 0 ((M_Cart+m)*g)/(M_Cart*l) 0];
B_Linear = [0;1/M_Cart;0;(-1)/(M_Cart*l)];
Q = 5*eye(4);
R = 20;

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
T = 10;
dt = 0.01;
X0 = [0;0;0.349066;0.174533];
t = 0;
X(:,1) = X0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);
    u = -K_reg*Xj;
    D1 = Pendulum_LQR(t,Xj,u);
    D2 = Pendulum_LQR(t+dt/2,Xj+D1*dt/2,u);
    D3 = Pendulum_LQR(t+dt/2,Xj+D2*dt/2,u);
    D4 = Pendulum_LQR(t+dt,Xj+D3*dt,u);
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    Time(k+1) = t + dt;
    k = k + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:));
title('LQR Controller Design Initial Conddition "0,0,20,10" and R equals to "20"')
xlabel('time(s)')
ylabel('x(m)')

subplot(4,1,2);plot(Time,X(2,:));
xlabel('time(s)')
ylabel('xdot(m/s)')

subplot(4,1,3);plot(Time,X(3,:));
xlabel('time(s)')
ylabel('theta(rad)')

subplot(4,1,4);plot(Time,X(4,:));
xlabel('time(s)')
ylabel('thetadot(rad/s)')