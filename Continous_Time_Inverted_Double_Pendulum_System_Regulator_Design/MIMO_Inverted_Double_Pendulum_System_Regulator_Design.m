%% Project#2_Advanced_Control_MIMO_Inverted_Double_Pendulum_System_Regulator_Design
clc 
clear 

%% System Paremeters
% g = 10, M_cart = 1kg, m_1 = 0.5kg, m_2 = 0.2kg, l_1 = 0.5m, l_2 = 0.5m 
A_Linear = [0 1 0 0 0 0;0 0 -7 0 0 0;0 0 0 1 0 0;0 0 42 0 -8 0;0 0 0 0 0 1;0 0 -28 0 28 0];
B_Linear = [0 0;1 -2;0 0;-2 12;0 0;0 -8];
C = [1 0 0 0 0 0;0 0 1 0 0 0];

%% Controllability and Observability
M = [B_Linear A_Linear*B_Linear A_Linear^2*B_Linear A_Linear^3*B_Linear A_Linear^4*B_Linear];
r_M = rank(M);
if r_M == min(size(M))
    
    fprintf('The system is controllable and the rank of M is\n')
    disp(r_M)
    
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
Amu1 = [A_Linear-mu1*eye(6) B_Linear];
Amu2 = [A_Linear-mu2*eye(6) B_Linear];
Amu3 = [A_Linear-mu3*eye(6) B_Linear];
Amu4 = [A_Linear-mu4*eye(6) B_Linear];
Amu5 = [A_Linear-mu5*eye(6) B_Linear];
Amu6 = [A_Linear-mu6*eye(6) B_Linear];

S1 = null(Amu1);
S2 = null(Amu2);
S3 = null(Amu3);
S4 = null(Amu4);
S5 = null(Amu5);
S6 = null(Amu6);

v1q1 = S1(:,1); % n=6, m=2, v=n*1, q=m*1
v2q2 = S2(:,2);
v3q3 = S3(:,1);
v4q4 = S4(:,2);
v5q5 = real(S5(:,1));
v6q6 = imag(S5(:,1));
v1 = v1q1(1:6); q1 = v2q2(7:8);
v2 = v2q2(1:6); q2 = v2q2(7:8);
v3 = v3q3(1:6); q3 = v3q3(7:8);
v4 = v4q4(1:6); q4 = v4q4(7:8);
v5 = v5q5(1:6); q5 = v5q5(7:8);
v6 = v6q6(1:6); q6 = v6q6(7:8);

v = [v1 v2 v3 v4 v5 v6];
r_v = rank(v);

if r_v == min(size(v))
    
    fprintf('The matrix v is invertable and its rank is\n')
    disp(r_v)
    
    vi = inv(v);
    K = -[q1 q2 q3 q4 q5 q6]*vi;
    fprintf('The Controller Gain "K" is\n')
    disp(K)
    
else
    
    disp('The v vectors must be selected independable')

end


%% Simulation
T = 10;
dt = 0.001;
X0 = [0;0.2;0.0872665;-0.0523599;0.174533;-0.0872665];
t = 0;
X(:,1) = X0;
Time(1) = t;
k = 1;
while t < T
    Xj = X(:,k);
    u = -K*Xj;
    D1 = MIMO_DOuble_Pendulum_Regul_Proj(t,Xj,u);
    D2 = MIMO_DOuble_Pendulum_Regul_Proj(t+dt/2,Xj+D1*dt/2,u);
    D3 = MIMO_DOuble_Pendulum_Regul_Proj(t+dt/2,Xj+D2*dt/2,u);
    D4 = MIMO_DOuble_Pendulum_Regul_Proj(t+dt,Xj+D3*dt,u);
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,k+1) = Xj;
    Time(k+1) = t + dt;
    k = k+1;
    t = t + dt;
end

%% Plots
figure;
subplot(6,1,1);plot(Time,X(1,:));
title('Regulator Controller Design')
xlabel('time(s)')
ylabel('x(m)')

subplot(6,1,2);plot(Time,X(2,:));
xlabel('time(s)')
ylabel('xdot(m/s)')

subplot(6,1,3);plot(Time,X(3,:));
xlabel('time(s)')
ylabel('theta1(rad)')

subplot(6,1,4);plot(Time,X(4,:));
xlabel('time(s)')
ylabel('theta1dot(rad/s)')

subplot(6,1,5);plot(Time,X(5,:));
xlabel('time(s)')
ylabel('theta2(rad)')

subplot(6,1,6);plot(Time,X(6,:));
xlabel('time(s)')
ylabel('theta2dot(rad/s)')