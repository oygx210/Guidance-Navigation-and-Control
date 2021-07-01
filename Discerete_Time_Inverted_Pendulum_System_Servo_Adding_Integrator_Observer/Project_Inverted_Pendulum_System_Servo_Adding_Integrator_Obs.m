%% Project#2_Advanced_Control_Inverted_Pendulum_System_Servo_Adding_Integrator_Observer
clc 
clear 

global M_Cart m g l C
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
Gh = [G zeros(4,1);-C 1];
Hh = [H;0];

mu_dcon = [-3 -3 -2-2i -2+2i -2];
mu_ddis = [exp(mu_dcon(1)*h) exp(mu_dcon(2)*h) exp(mu_dcon(3)*h) exp(mu_dcon(4)*h) exp(mu_dcon(5)*h)];
fprintf('The closed loop discerete time system desired eigen values must be placed at\n')
disp(mu_ddis)

muo_dcon = [-10 -10 -10 -10];
muo_ddis = [exp(muo_dcon(1)*h) exp(muo_dcon(2)*h) exp(muo_dcon(3)*h) exp(muo_dcon(4)*h)];
fprintf('The closed loop discerete time approximated system desired eigen values must be placed at\n')
disp(muo_ddis)

%% Designing Controller Gain, Ackerman Method
Mh_Discerete = [Hh Gh*Hh Gh^2*Hh Gh^3*Hh Gh^4*Hh];
r_Mh_Discerete = rank(Mh_Discerete);

if r_Mh_Discerete == min(size(Mh_Discerete))
    
    fprintf('The discerete time system is controllable and the rank of Mh is\n')
    disp(r_Mh_Discerete)
    
    K = acker(Gh,Hh,mu_ddis);

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
T = 400;
dt = 0.001;
X0 = [0;0;0.017/2;0.008/2;0];
Xh0 = [0.001/2;0.01/2;0.01/2;0.001/2];
t = 0;
Time(1) = t;
k = 0;
i = 1;
X(:,i) = X0;
Xh(:,1) = Xh0;

while t < T
    
    Xj = X(:,i);
    y = C*Xj(1:4);
    if mod(i,floor(h/dt))==1
        
        k=k+1;
        ud(k)=-K(1:4)*Xh(:,k) - K(5)*Xj(5);
        yh = C*Xh(:,k);
        Xh(:,k+1) = G*Xh(:,k) + H*ud(k) + Ko*(y-yh);
    end
    
    u(:,i) = ud(k);
    Xhj = Xh(:,k);
    Xh(:,i) = Xhj;
    yr(i) = 0.5*sign(sin(0.02*t));
    
    D1 = Pendulum_Servo_Add_Int_Obs_Proj(t,Xj,u(i),yr(i));
    D2 = Pendulum_Servo_Add_Int_Obs_Proj(t+dt/2,Xj+D1*dt/2,u(i),yr(i));
    D3 = Pendulum_Servo_Add_Int_Obs_Proj(t+dt/2,Xj+D2*dt/2,u(i),yr(i));
    D4 = Pendulum_Servo_Add_Int_Obs_Proj(t+dt,Xj+D3*dt,u(i),yr(i));   
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,i+1) = Xj;
    Time(i+1) = t + dt;
    i = i + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(5,1,1);plot(Time,X(1,:),Time(1:end-1),Xh(1,:),'g',Time(1:end-1),yr,'r');
title('Servo With Observer Design and Smapling Time of "0.05"')
xlabel('time(s)')
ylabel('x(m)')
legend('X','Xhat','Yr','location','northeast')

subplot(5,1,2);plot(Time,X(2,:),Time(1:end-1),Xh(2,:),'g');
xlabel('time(s)')
ylabel('xdot(m/s)')
legend('X','Xhat','location','northeast')

subplot(5,1,3);plot(Time,X(3,:),Time(1:end-1),Xh(3,:),'g');
xlabel('time(s)')
ylabel('theta(rad)')
legend('X','Xhat','location','northeast')

subplot(5,1,4);plot(Time,X(4,:),Time(1:end-1),Xh(4,:),'g');
xlabel('time(s)')
ylabel('thetadot(rad/s)')
legend('X','Xhat','location','northeast')

subplot(5,1,5);plot(Time(1:end-1),u);
xlabel('time(s)')
ylabel('ZOH Input')