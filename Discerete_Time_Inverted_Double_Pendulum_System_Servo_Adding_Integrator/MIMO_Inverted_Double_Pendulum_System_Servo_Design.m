%% Project#2_Advanced_Control_MIMO_Inverted_Double_Pendulum_System_Servo_Design
clc 
clear 
global C
%% System Paremeters
% g = 10, M_cart = 1kg, m_1 = 0.5kg, m_2 = 0.2kg, l_1 = 0.5m, l_2 = 0.5m 
A_Linear = [0 1 0 0 0 0;0 0 -7 0 0 0;0 0 0 1 0 0;0 0 42 0 -8 0;0 0 0 0 0 1;0 0 -28 0 28 0];
B_Linear = [0 0;1 -2;0 0;-2 12;0 0;0 -8];
C = [1 0 0 0 0 0;0 0 1 0 0 0];
h = 0.05;

G = expm(A_Linear*h);
H = (eye(6) + (A_Linear*h)/2 + ((A_Linear^2)*(h^2))/6 + ((A_Linear^3)*(h^3))/24)*h*B_Linear;
Gh = [G zeros(6,2);-C eye(2)];
Hh = [H;zeros(2,2)];

mu_dcon = [-4.01 -3.99 -3.01 -2.99 -2-2i -2+2i -6.01 -5.99];
mu_ddis = [exp(mu_dcon(1)*h) exp(mu_dcon(2)*h) exp(mu_dcon(3)*h) exp(mu_dcon(4)*h) exp(mu_dcon(5)*h) exp(mu_dcon(6)*h) exp(mu_dcon(7)*h) exp(mu_dcon(8)*h)];
fprintf('The closed loop discerete time system desired eigen values must be placed at\n')
disp(mu_ddis)


%% Designing Controller Gain, Place Method
Mh_Discerete = [Hh Gh*Hh Gh^2*Hh Gh^3*Hh Gh^4*Hh Gh^5*Hh Gh^6*Hh];
r_Mh_Discerete = rank(Mh_Discerete);

if r_Mh_Discerete == min(size(Mh_Discerete))
    
    fprintf('The discerete time system is controllable and the rank of Mh is\n')
    disp(r_Mh_Discerete)
    
    K = place(Gh,Hh,mu_ddis);

    fprintf('The Controller Gain "K" is\n')
    disp(K)

else 
    disp('The continous time system is unctrollable')
end

%% Simulation
T = 60;
dt = 0.001;
X0 = [0;0.2;0.0872665;-0.0523599;0.174533;-0.0872665;0;0];
t = 0;
Time(1) = t;
k = 0;
i = 1;
X(:,1) = X0;

while t < T
    
     Xj = X(:,i);
    
    if mod(i,floor(h/dt))==1
        
        k=k+1;
        ud = -K*Xj;
        ud(:,k) = ud;
        
    end
    
    
    yr1(i)=1*sign(sin(0.25*t));
    yr2(i)=(0.0523599)*sign(sin(0.5*t));
    
    uj = ud(:,k);
    u(:,i) = uj;
    
    D1=MIMO_Double_Pendulum_Servo_Proj(t,Xj,uj,yr1(i),yr2(i));
    D2=MIMO_Double_Pendulum_Servo_Proj(t+dt/2,Xj+D1*dt/2,uj,yr1(i),yr2(i));
    D3=MIMO_Double_Pendulum_Servo_Proj(t+dt/2,Xj+D2*dt/2,uj,yr1(i),yr2(i));
    D4=MIMO_Double_Pendulum_Servo_Proj(t+dt,Xj+D3*dt,uj,yr1(i),yr2(i));
    Xj=Xj+(D1+2*D2+2*D3+D4)/6*dt;
    X(:,i+1) = Xj;
    Time(i+1)= t + dt;
    i = i+1;
    t = t+dt;
end

%% Plots
figure;
subplot(6,1,1);plot(Time,X(1,:),Time(1:end-1),yr1,'g');
title('Servo Design Signal Frequencies "0.25 & 0.5" and Smapling Time "0.05"')
xlabel('time(s)')
ylabel('X(m)')
legend('Y1','Yr1','location','northeast')

subplot(6,1,2);plot(Time,X(2,:));
xlabel('time(s)')
ylabel('Xdot(m/s)')

subplot(6,1,3);plot(Time,X(3,:),Time(1:end-1),yr2,'g');
xlabel('time(s)')
ylabel('theta1(rad)')
legend('Y2','Yr2','location','northeast')

subplot(6,1,4);plot(Time,X(4,:));
xlabel('time(s)')
ylabel('theta1dot(rad/s)')

subplot(6,1,5);plot(Time,X(5,:));
xlabel('time(s)')
ylabel('theta2(rad)')

subplot(6,1,6);plot(Time,X(6,:));
xlabel('time(s)')
ylabel('theta2dot(rad/s)')