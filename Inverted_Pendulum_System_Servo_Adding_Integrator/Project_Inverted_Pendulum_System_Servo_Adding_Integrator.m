%% Project#1_Advanced_Control_Inverted_Pendulum_System_Servo_Adding_Integrator
clc 
clear 

global M_Cart m g l C_Yx
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

%% Simulation
T = 60;
dt = 0.01;
X0 = [0;0;0.349066;0.174533;0];
t = 0;
X(:,1) = X0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);
    yr(k) = 0.5*sign(sin(0.2*t));
    u = -Kh_srv*Xj;
    D1 = Pendulum_Servo_Add_Int_Proj(t,Xj,u,yr(k));
    D2 = Pendulum_Servo_Add_Int_Proj(t+dt/2,Xj+D1*dt/2,u,yr(k));
    D3 = Pendulum_Servo_Add_Int_Proj(t+dt/2,Xj+D2*dt/2,u,yr(k));
    D4 = Pendulum_Servo_Add_Int_Proj(t+dt,Xj+D3*dt,u,yr(k));
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    Time(k+1) = t + dt;
    k = k + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:),Time(1:end-1),yr,'g');
title('Adding Integrator Servo Design Initial Conddition "0,0,10,5"')
xlabel('time(s)')
ylabel('x(m)')
legend('Y','Yr','location','northeast')

subplot(4,1,2);plot(Time,X(2,:));
xlabel('time(s)')
ylabel('xdot(m/s)')

subplot(4,1,3);plot(Time,X(3,:));
xlabel('time(s)')
ylabel('theta(rad)')

subplot(4,1,4);plot(Time,X(4,:));
xlabel('time(s)')
ylabel('thetadot(rad/s)')