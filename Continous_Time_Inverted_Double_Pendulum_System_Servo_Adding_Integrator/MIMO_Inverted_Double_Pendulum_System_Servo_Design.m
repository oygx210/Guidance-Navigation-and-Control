%% Project#2_Advanced_Control_MIMO_Inverted_Double_Pendulum_System_Servo_Design
clc 
clear 
global C
%% System Paremeters
% g = 10, M_cart = 1kg, m_1 = 0.5kg, m_2 = 0.2kg, l_1 = 0.5m, l_2 = 0.5m 
A_Linear = [0 1 0 0 0 0;0 0 -7 0 0 0;0 0 0 1 0 0;0 0 42 0 -8 0;0 0 0 0 0 1;0 0 -28 0 28 0];
B_Linear = [0 0;1 -2;0 0;-2 12;0 0;0 -8];
C = [1 0 0 0 0 0;0 0 1 0 0 0];
Ah_Linear = [A_Linear zeros(6,2);-C zeros(2,2)]; %n=8, m=2
Bh_Linear = [B_Linear;zeros(2,2)];
%% Controllability and Observability
Mh = [Bh_Linear Ah_Linear*Bh_Linear Ah_Linear^2*Bh_Linear Ah_Linear^3*Bh_Linear Ah_Linear^4*Bh_Linear Ah_Linear^5*Bh_Linear Ah_Linear^6*Bh_Linear];
r_Mh = rank(Mh);
if r_Mh == min(size(Mh))
    
    fprintf('The system is controllable and the rank of Mh is\n')
    disp(r_Mh)
    
else
    
    disp('The System is Uncontrollable')

end

N = [C;C*A_Linear;C*A_Linear^2;C*A_Linear^3;C*A_Linear^4;C*A_Linear^5];
r_N = rank(N);
if r_N == min(size(N))
    
    fprintf('The system is observable and the rank of N is\n')
    disp(r_N)
    
else
    
    disp('The System is Unobservable')

end

%% Designing Controller Gain, EESA Method
mu1 = -4;
mu2 = -4;
mu3 = -3;
mu4 = -3;
mu5 = -2+1i;
mu6 = -2-1i;
mu7 = -6;
mu8 = -6;
Amu1 = [Ah_Linear-mu1*eye(8) Bh_Linear];
Amu2 = [Ah_Linear-mu2*eye(8) Bh_Linear];
Amu3 = [Ah_Linear-mu3*eye(8) Bh_Linear];
Amu4 = [Ah_Linear-mu4*eye(8) Bh_Linear];
Amu5 = [Ah_Linear-mu5*eye(8) Bh_Linear];
Amu6 = [Ah_Linear-mu6*eye(8) Bh_Linear];
Amu7 = [Ah_Linear-mu7*eye(8) Bh_Linear];
Amu8 = [Ah_Linear-mu8*eye(8) Bh_Linear];

S1 = null(Amu1);
S2 = null(Amu2);
S3 = null(Amu3);
S4 = null(Amu4);
S5 = null(Amu5);
S6 = null(Amu6);
S7 = null(Amu7);
S8 = null(Amu8);

v1q1 = S1(:,1); % n=8, m=2, v=n*1, q=m*1
v2q2 = S2(:,2);
v3q3 = S3(:,1);
v4q4 = S4(:,2);
v5q5 = real(S5(:,1));
v6q6 = imag(S5(:,1));
v7q7 = S7(:,1);
v8q8 = S8(:,2);

v1 = v1q1(1:8); q1 = v2q2(9:10);
v2 = v2q2(1:8); q2 = v2q2(9:10);
v3 = v3q3(1:8); q3 = v3q3(9:10);
v4 = v4q4(1:8); q4 = v4q4(9:10);
v5 = v5q5(1:8); q5 = v5q5(9:10);
v6 = v6q6(1:8); q6 = v6q6(9:10);
v7 = v7q7(1:8); q7 = v7q7(9:10);
v8 = v8q8(1:8); q8 = v8q8(9:10);

v = [v1 v2 v3 v4 v5 v6 v7 v8];
r_v = rank(v);

if r_v == min(size(v))
    
    fprintf('The matrix v is invertable and its rank is\n')
    disp(r_v)
    
    vi = inv(v);
    K = -[q1 q2 q3 q4 q5 q6 q7 q8]*vi;
    fprintf('The Controller Gain "K" is\n')
    disp(K)
    
else
    
    disp('The v vectors must be selected independable')

end

%% Simulation
T = 60;
dt = 0.001;
X0 = [0;0.2;0.0872665;-0.0523599;0.174533;-0.0872665;0;0];
t = 0;
X(:,1) = X0;
Time(1) = t;
k = 1;
while t < T
    Xj=X(:,k);
    yr1(k)=1*sign(sin(0.25*t));
    yr2(k)=(0.0523599)*sign(sin(0.5*t));
    u=-K*Xj;
    D1=MIMO_Double_Pendulum_Servo_Proj(t,Xj,u,yr1(k),yr2(k));
    D2=MIMO_Double_Pendulum_Servo_Proj(t+dt/2,Xj+D1*dt/2,u,yr1(k),yr2(k));
    D3=MIMO_Double_Pendulum_Servo_Proj(t+dt/2,Xj+D2*dt/2,u,yr1(k),yr2(k));
    D4=MIMO_Double_Pendulum_Servo_Proj(t+dt,Xj+D3*dt,u,yr1(k),yr2(k));
    Xj=Xj+(D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1)=Xj;
    Time(k+1)=t+dt;
    k=k+1;
    t=t+dt;
end

%% Plots
figure;
subplot(6,1,1);plot(Time,X(1,:),Time(1:end-1),yr1,'g');
title('Servo Design ')
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