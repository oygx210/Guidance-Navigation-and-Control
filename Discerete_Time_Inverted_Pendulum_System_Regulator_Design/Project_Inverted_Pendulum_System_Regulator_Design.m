%% Project#2_Advanced_Control_Inverted_Pendulum_System_Regulator_Design
clc 
clear 

global M_Cart m g l
%% System Parameters
M_Cart = 2; %% Cart Mass
m = 0.5; %% Pendulum Mass
l = 1;  %% Pendulum Beam Length
g = 9.81;
h = 0.05;

A_Linear = [0 1 0 0;0 0 (-m*g)/(M_Cart) 0;0 0 0 1; 0 0 ((M_Cart+m)*g)/(M_Cart*l) 0];
B_Linear = [0;1/M_Cart;0;(-1)/(M_Cart*l)];
C = [1 0 0 0];
G = expm(A_Linear*h);
H = (eye(4) + (A_Linear*h)/2 + ((A_Linear^2)*(h^2))/6 + ((A_Linear^3)*(h^3))/24)*h*B_Linear;

mu_dcon = [-3 -3 -2+2i -2-2i];
mu_ddis = [exp(mu_dcon(1)*h) exp(mu_dcon(2)*h) exp(mu_dcon(3)*h) exp(mu_dcon(4)*h)];
fprintf('The closed loop discerete time system desired eigen values must be placed at\n')
disp(mu_ddis)

muo_dcon = [-10 -10 -10 -10];
muo_ddis = [exp(muo_dcon(1)*h) exp(muo_dcon(2)*h) exp(muo_dcon(3)*h) exp(muo_dcon(4)*h)];
fprintf('The closed loop discerete time approximated system desired eigen values must be placed at\n')
disp(muo_ddis)

%% Designing Controller Gain, Ackerman Method
M_Continous = [B_Linear A_Linear*B_Linear A_Linear^2*B_Linear A_Linear^3*B_Linear];
r_M_Continous = rank(M_Continous);
M_Discerete = [H G*H G^2*H G^3*H];
r_M_Discerete = rank(M_Discerete);

if r_M_Continous == min(size(M_Continous))
    
    fprintf('The contionous time system is controllable and the rank of M is\n')
    disp(r_M_Continous)
    

else 
    disp('The continous time system is unctrollable')
end

if r_M_Discerete == min(size(M_Discerete))
    
    fprintf('The discerete time system is controllable and the rank of M is\n')
    disp(r_M_Discerete)
    
    K = acker(G,H,mu_ddis);

    fprintf('The Controller Gain "K" is\n')
    disp(K)

else 
    disp('The continous time system is unctrollable')
end

%% Designing Observer Gain, Ackerman Method
N_T_Contionous = [C' A_Linear'*C' (A_Linear')^2*C' (A_Linear')^3*C'];
r_N_Contionous = rank(N_T_Contionous);
N_T_Discerete = [C' G'*C' (G')^2*C' (G')^3*C']; 
r_N_Discerete = rank(N_T_Discerete);

if r_N_Contionous == min(size(N_T_Contionous))
    
    fprintf('The contiouns time system is Observable and the rank of N_Transpose is\n')
    disp(r_N_Contionous)
    
else 
    disp('The continous time system is unobservable')
end

if r_N_Discerete == min(size(N_T_Discerete))
    
    fprintf('The discerete time system is Observable and the rank of N_Transpose is\n')
    disp(r_N_Discerete)
    
    Ko = acker(G',C',muo_ddis);
    Ko = Ko';

    fprintf('The observer gain "Ko" is\n')
    disp(Ko)
    
else 
    disp('The discerete time system is unobservable')
end

%% Simulation
T = 5;
dt = 0.001;
X0 = [0;0;0.174533;0.0872665];
t = 0;
Time(1) = t;
k = 0;
i = 1;
X(:,i) = X0;
while t < T
    Xj = X(:,i);
    
    if mod(i,floor(h/dt))==1
        k=k+1;
        ud(k)=-K*Xj;
    end
    
    u(:,i) = ud(k);
    D1 = Pendulum_Regul_Proj(t,Xj,u(i));
    D2 = Pendulum_Regul_Proj(t+dt/2,Xj+D1*dt/2,u(i));
    D3 = Pendulum_Regul_Proj(t+dt/2,Xj+D2*dt/2,u(i));
    D4 = Pendulum_Regul_Proj(t+dt,Xj+D3*dt,u(i));
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,i+1) = Xj;
    Time(i+1) = t + dt;
    i = i+1;
    t = t + dt;
end

%% Plots
figure;
subplot(5,1,1);plot(Time,X(1,:));
title('Regulator Controller Design Initial Condition "0,0,10,5" and Smapling Time of "0.05"')
xlabel('time(s)')
ylabel('x(m)')

subplot(5,1,2);plot(Time,X(2,:));
xlabel('time(s)')
ylabel('xdot(m/s)')

subplot(5,1,3);plot(Time,X(3,:));
xlabel('time(s)')
ylabel('theta(rad)')

subplot(5,1,4);plot(Time,X(4,:));
xlabel('time(s)')
ylabel('thetadot(rad/s)')

subplot(5,1,5);plot(Time(1:end-1),u);
xlabel('time(s)')
ylabel('ZOH Input')