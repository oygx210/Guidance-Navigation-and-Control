%% Project#1_Advanced_Control_Inverted_Pendulum_System_Servo_Feed_Forward
clc 
clear 

global M_Cart m g l A B
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
    K_srv = acker(A_Linear,B_Linear,mu_d);

    fprintf('The Controller Gain "K" is\n')
    disp(K_srv)
else 
    disp('The System is Unctrollable')
end


%% Simulation
T = 60;
dt = 0.01;
X0 = [0;0;0.349066;0.174533];
t = 0;
X(:,1) = X0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);
    y = C_Yx*Xj;
    yr(k) = 0.5*sign(sin(0.2*t));
    uff = (-yr(k))/(C_Yx*inv(A-B*K_srv)*B);
    u = -K_srv*Xj + uff;
    D1 = Pendulum_Servo_FF_Proj(t,Xj,u);
    D2 = Pendulum_Servo_FF_Proj(t+dt/2,Xj+D1*dt/2,u);
    D3 = Pendulum_Servo_FF_Proj(t+dt/2,Xj+D2*dt/2,u);
    D4 = Pendulum_Servo_FF_Proj(t+dt,Xj+D3*dt,u);   
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    
    Time(k+1) = t + dt;
    k = k + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(4,1,1);plot(Time,X(1,:),Time(1:end-1),yr,'g');
title('Feed Forward Servo Controller Design Initial Conddition "0,0,20,10"')
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