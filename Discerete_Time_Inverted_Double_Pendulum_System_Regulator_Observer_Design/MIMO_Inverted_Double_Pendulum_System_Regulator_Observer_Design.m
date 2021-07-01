%% Project#2_Advanced_Control_MIMO_Inverted_Double_Pendulum_System_Regulator_Design
clc 
clear 

%% System Paremeters
% g = 10, M_cart = 1kg, m_1 = 0.5kg, m_2 = 0.2kg, l_1 = 0.5m, l_2 = 0.5m 
A_Linear = [0 1 0 0 0 0;0 0 -7 0 0 0;0 0 0 1 0 0;0 0 42 0 -8 0;0 0 0 0 0 1;0 0 -28 0 28 0];
B_Linear = [0 0;1 -2;0 0;-2 12;0 0;0 -8];
C = [1 0 0 0 0 0;0 0 1 0 0 0];

h = 0.05;
G = expm(A_Linear*h);
H = (eye(6) + (A_Linear*h)/2 + ((A_Linear^2)*(h^2))/6 + ((A_Linear^3)*(h^3))/24)*h*B_Linear;

mu_dcon = [-3.01 -2.99 -1.01 -1.99 -2+1i -2-1i];
mu_ddis = [exp(mu_dcon(1)*h) exp(mu_dcon(2)*h) exp(mu_dcon(3)*h) exp(mu_dcon(4)*h) exp(mu_dcon(5))*h exp(mu_dcon(6))*h];
fprintf('The closed loop discerete time system desired eigen values must be placed at\n')
disp(mu_ddis)

muo_dcon = [-10 -10.01 -10.02 -10.03 -9.99 -9.98];
muo_ddis = [exp(muo_dcon(1)*h) exp(muo_dcon(2)*h) exp(muo_dcon(3)*h) exp(muo_dcon(4)*h) exp(muo_dcon(5)*h) exp(muo_dcon(6)*h)];
fprintf('The closed loop discerete time approximated system desired eigen values must be placed at\n')
disp(muo_ddis)

%% Designing Controller Gain, Place Method
M_Continous = [B_Linear A_Linear*B_Linear A_Linear^2*B_Linear A_Linear^3*B_Linear A_Linear^4*B_Linear];
r_M_Continous = rank(M_Continous);
M_Discerete = [H G*H G^2*H G^3*H G^4*H];
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
    
    K = place(G,H,mu_ddis);

    fprintf('The Controller Gain "K" is\n')
    disp(K)

else 
    disp('The continous time system is unctrollable')
end

%% Designing Observer Gain, Place Method
N_T_Contionous = [C' A_Linear'*C' (A_Linear')^2*C' (A_Linear')^3*C' (A_Linear')^4*C'];
r_N_Contionous = rank(N_T_Contionous);
N_T_Discerete = [C' G'*C' (G')^2*C' (G')^3*C' (G')^4*C']; 
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
    
    Ko = place(G',C',muo_ddis);
    Ko = Ko';

    fprintf('The observer gain "Ko" is\n')
    disp(Ko)
    
else 
    disp('The discerete time system is unobservable')
end

%% Simulation
T = 10;
dt = 0.001;
X0 = [0;0.02/2;0.008/2;-0.005/2;0.017/2;-0.008/2];
Xh0 = [0.0001;0.0001;0.0001;0.0001;0.0001;0.0001];
t = 0;
Time(1) = t;
k = 0;
i = 1;
X(:,i) = X0;
Xh(:,1) = Xh0;

while t < T
    
    Xj = X(:,i);
    y = C*Xj;
    
    if mod(i,floor(h/dt))==1
        k=k+1;
        ud = -K*Xh(:,k);
        ud(:,k) = ud;
        yh = C*Xh(:,k);
        Xh(:,k+1) = G*Xh(:,k) + H*ud(:,k) + Ko*(y-yh);
       
    end
    
    uj = ud(:,k);
    u(:,i) = uj;
    Xhj = Xh(:,k);
    Xh(:,i) = Xhj;

    D1 = MIMO_Double_Pendulum_Regul_Obs_Proj(t,Xj,uj);
    D2 = MIMO_Double_Pendulum_Regul_Obs_Proj(t+dt/2,Xj+D1*dt/2,uj);
    D3 = MIMO_Double_Pendulum_Regul_Obs_Proj(t+dt/2,Xj+D2*dt/2,uj);
    D4 = MIMO_Double_Pendulum_Regul_Obs_Proj(t+dt,Xj+D3*dt,uj);   
    Xj = Xj + (D1+2*D2+2*D3+D4)/6*dt;
    X(:,i+1) = Xj;
    Time(i+1) = t + dt;
    i = i + 1;
    t = t + dt;
end

%% Plots
figure;
subplot(6,1,1);plot(Time,X(1,:),Time(1:end-1),Xh(1,:),'g');
title('Regulator Controller With Observer Design and Samling time "0.05"')
xlabel('time(s)')
ylabel('x(m)')

subplot(6,1,2);plot(Time,X(2,:),Time(1:end-1),Xh(2,:),'g');
xlabel('time(s)')
ylabel('xdot(m/s)')

subplot(6,1,3);plot(Time,X(3,:),Time(1:end-1),Xh(3,:),'g');
xlabel('time(s)')
ylabel('theta1(rad)')

subplot(6,1,4);plot(Time,X(4,:),Time(1:end-1),Xh(4,:),'g');
xlabel('time(s)')
ylabel('theta1dot(rad/s)')

subplot(6,1,5);plot(Time,X(5,:),Time(1:end-1),Xh(5,:),'g');
xlabel('time(s)')
ylabel('theta2(rad)')

subplot(6,1,6);plot(Time,X(6,:),Time(1:end-1),Xh(6,:),'g');
xlabel('time(s)')
ylabel('theta2dot(rad/s)')