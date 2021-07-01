%% Project#1_Advanced_Control_Inverted_Pendulum_System_Servo_Adding_Integrator_Minimum_Order_Observer
clc 
clear 

global M_Cart m g l C_Yx Ko_Yx
%% System Parameters
M_Cart = 2; %% Cart Mass
m = 0.5; %% Pendulum Mass
l = 1;  %% Pendulum Beam Length
g = 9.81;

A_Linear = [0 1 0 0;0 0 (-m*g)/(M_Cart) 0;0 0 0 1; 0 0 ((M_Cart+m)*g)/(M_Cart*l) 0];
B_Linear = [0;1/M_Cart;0;(-1)/(M_Cart*l)];
C_Yx = [1 0 0 0];
Ah_Linear = [A_Linear zeros(4,1);-C_Yx 0];
Bh_Linear = [B_Linear;0];
Aaa_Linear = A_Linear(1);
Aab_Linear = A_Linear(1,2:4);
Aba_Linear = A_Linear(2:4,1);
Abb_Linear = [A_Linear(2,2:4);A_Linear(3,2:4);A_Linear(4,2:4)];
Ba_Linear = B_Linear(1);
Bb_Linear = B_Linear(2:4,1);
%% Designing Controller Gain, Ackerman Method
Mh = [Bh_Linear Ah_Linear*Bh_Linear Ah_Linear^2*Bh_Linear Ah_Linear^3*Bh_Linear Ah_Linear^4*Bh_Linear];
r_Mh = rank(Mh);
if r_Mh == min(size(Mh))
    fprintf('The system is controllable and the rank of Mh is\n')
    disp(r_Mh)
    
    mu_d = [-2+2i -2-2i -3 -3 -2]; %% Desired Eigenvalues
    Kh_srv = acker(Ah_Linear,Bh_Linear,mu_d);

    fprintf('The Controller Gain "K" is\n')
    disp(Kh_srv)
else 
    disp('The System is Unctrollable')
end

%% Designing Observer Gain When System Observes X1=x, Ackerman Method
N_T_Yx = [Aab_Linear' Abb_Linear'*Aab_Linear' (Abb_Linear')^2*Aab_Linear']; %% N tranpose matrix when system observs X1=x
r_N_T_Yx = rank(N_T_Yx);
if r_N_T_Yx == min(size(N_T_Yx))
    fprintf('When system observes (x), it is Observable and the rank of N_Transpose is\n')
    disp(r_N_T_Yx)
    
    muo_d = [-10 -10 -10];
    Ko_T_Yx = acker(Abb_Linear',Aab_Linear',muo_d);
    Ko_Yx = Ko_T_Yx';

    fprintf('The Observer Gain "Ko" When System Observes (x) is\n')
    disp(Ko_Yx)
else 
    disp('The system is unobservable when it observes (x)')
end

%% Simulation
T = 60;
dt = 0.0001;
X0 = [0;0;0.349066;0.174533;0];
Xbh0 = [0.01;0.1;0.1];
Eth0 = Xbh0 - Ko_Yx*X0(1);
t = 0;
X(:,1) = X0;
Xbh(:,1) = Xbh0;
Eth(:,1) = Eth0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);   
    Ethj = Eth(:,k);
    Xbhj = Ethj + Ko_Yx*Xj(1);
    y = C_Yx*Xj(1:4); %%%% y=Xa=X1;
    yr(k) = 0.5*sign(sin(0.2*t));
    u = -Kh_srv(1)*Xj(1) - Kh_srv(2:4)*Xbhj - Kh_srv(5)*Xj(5);
    D1 = Pendulum_Servo_Add_Int_MO_Obs_Proj(t,Xj,u,yr(k));
    D2 = Pendulum_Servo_Add_Int_MO_Obs_Proj(t+dt/2,Xj+D1*dt/2,u,yr(k));
    D3 = Pendulum_Servo_Add_Int_MO_Obs_Proj(t+dt/2,Xj+D2*dt/2,u,yr(k));
    D4 = Pendulum_Servo_Add_Int_MO_Obs_Proj(t+dt,Xj+D3*dt,u,yr(k));   
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    O1 = Pendulum_Min_Obser_Servo_Add_Int_Proj(t,Ethj,u,y);
    O2 = Pendulum_Min_Obser_Servo_Add_Int_Proj(t+dt/2,Ethj+O1*dt/2,u,y);
    O3 = Pendulum_Min_Obser_Servo_Add_Int_Proj(t+dt/2,Ethj+O2*dt/2,u,y);
    O4 = Pendulum_Min_Obser_Servo_Add_Int_Proj(t+dt,Ethj+O3*dt,u,y);   
    Ethj = Ethj+(O1+2*O2+2*O3+O4)/6*dt;
    Eth(:,k+1) = Ethj;
    Xbh(:,k+1) = Ethj + Ko_Yx*Xj(1);
    Time(k+1) = t + dt;
    k = k + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:),Time(1:end-1),yr,'r');
title('Integrator Servo Design with Minimum Order Observer Initial Conddition "0,0,10,5"')
xlabel('time(s)')
ylabel('x(m)')
legend('X','Yr','location','northeast')

subplot(4,1,2);plot(Time,X(2,:),Time,Xbh(1,:),'g');
xlabel('time(s)')
ylabel('xdot(m/s)')
legend('X','Xbh','location','northeast')

subplot(4,1,3);plot(Time,X(3,:),Time,Xbh(2,:),'g');
xlabel('time(s)')
ylabel('theta(rad)')
legend('X','Xbh','location','northeast')

subplot(4,1,4);plot(Time,X(4,:),Time,Xbh(3,:),'g');
xlabel('time(s)')
ylabel('thetadot(rad/s)')
legend('X','Xbh','location','northeast')