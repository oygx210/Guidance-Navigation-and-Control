%% Project#1_Advanced_Control_Inverted_Pendulum_System_Regulator_Full_Observer_Design
clc 
clear 

global M_Cart m g l Ko_Yx C_Yx
%% System Parameters
M_Cart = 2; %% Cart Mass
m = 0.5; %% Pendulum Mass
l = 1;  %% Pendulum Beam Length
g = 9.81;

A_Linear = [0 1 0 0;0 0 (-m*g)/(M_Cart) 0;0 0 0 1; 0 0 ((M_Cart+m)*g)/(M_Cart*l) 0];
B_Linear = [0;1/M_Cart;0;(-1)/(M_Cart*l)];
C_Yx = [1 0 0 0];

%% Designing Controller Gain, Ackerman Method
M = [B_Linear A_Linear*B_Linear A_Linear^2*B_Linear A_Linear^3*B_Linear];
r_M = rank(M);
if r_M == min(size(M))
    fprintf('The system is controllable and the rank of M is\n')
    disp(r_M)
    
    mu_d = [-3 -3 -2+2i -2-2i]; %% Desired Eigenvalues
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
    
    muo_d = [-10 -10 -10 -10];
    Ko_T_Yx = acker(A_Linear',C_Yx',muo_d);
    Ko_Yx = Ko_T_Yx';

    fprintf('The Observer Gain "Ko" When System Observes (x) is\n')
    disp(Ko_Yx)
else 
    disp('The system is unobservable when it observes (x)')
end

%% Simulation
T = 5;
dt = 0.001;
X0 = [0;0;0.349066;0.174533];
Xh0 = [0.01;0.01;0.1;0.1];
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
    D1 = Pendulum_Regul_FO_Obs_Proj(t,Xj,u);
    D2 = Pendulum_Regul_FO_Obs_Proj(t+dt/2,Xj+D1*dt/2,u);
    D3 = Pendulum_Regul_FO_Obs_Proj(t+dt/2,Xj+D2*dt/2,u);
    D4 = Pendulum_Regul_FO_Obs_Proj(t+dt,Xj+D3*dt,u);   
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    O1 = Pendulum_Full_Obser_Regual_Proj(t,Xhj,u,y);
    O2 = Pendulum_Full_Obser_Regual_Proj(t+dt/2,Xhj+O1*dt/2,u,y);
    O3 = Pendulum_Full_Obser_Regual_Proj(t+dt/2,Xhj+O2*dt/2,u,y);
    O4 = Pendulum_Full_Obser_Regual_Proj(t+dt,Xhj+O3*dt,u,y);   
    Xhj = Xhj + (O1+2*O2+2*O3+O4)/6*dt;
    Xh(:,k+1) = Xhj;
    Time(k+1) = t + dt;
    k = k + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:),Time,Xh(1,:),'g');
title('Regulator With Full Order ObserverController Design Initial Conddition "0,0,20,10"')
xlabel('time(s)')
ylabel('x(m)')
legend('X','Xhat','location','northeast')

subplot(4,1,2);plot(Time,X(2,:),Time,Xh(1,:),'g');
xlabel('time(s)')
ylabel('xdot(m/s)')
legend('X','Xhat','location','northeast')

subplot(4,1,3);plot(Time,X(3,:),Time,Xh(1,:),'g');
xlabel('time(s)')
ylabel('theta(rad)')
legend('X','Xhat','location','northeast')

subplot(4,1,4);plot(Time,X(4,:),Time,Xh(1,:),'g');
xlabel('time(s)')
ylabel('thetadot(rad/s)')
legend('X','Xhat','location','northeast')