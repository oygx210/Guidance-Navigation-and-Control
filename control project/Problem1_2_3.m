%% Control System Program Problem 1-2 1-3 2-1
format long 
format compact
%% Variables
t = 41; %working time
m_0 = 4000; %initial mass
K = -3000; %thrust coeff
m_dot = -41; %mass rate
d = 0.7; %missile diameter
S = 3.14 * 0.25 * d^2; %missile area
ro_0 = 1.2251; %density at sea level
const = 7 * 10^(-6);
c_d = 0.4; %drag coeff.
k = 6371000; %earth redius
g_0 = 9.8; %gravity coeff.
h = 18690 ; %hight at working time
V = 874.3 ;% velocity at working time
%% Matrix
A = [0 , 1 , 0;(1/(m_0+m_dot*t))*(0.5*ro_0*S*c_d*(V^2)*const)+((2*g_0*k^2)/(k+h)^3) , (-1/(m_0 + m_dot * t)) * (0.5*ro_0*S*c_d*(1-const*h)*V) , (-K*m_dot)/((m_0+m_dot*t)^2)+((0.5*ro_0*S*c_d*V^2*(1-const*h))/((m_0+m_dot*t)^2));0 , 0 , 0] ;
B = [0 ; K / (m_0+m_dot*t) ; 1];
C = [1 , 0 , 0];
D = [0] ; 
%% Open Loop Transfer Function (Problem 1-2)
[num_open , den_open] = ss2tf(A,B,C,D)
g_poen = tf(num_open , den_open)
%% Problem 1-3 Plot 
t_0=0:0.01:41;
 figure,plot(t_0,Linear_Hight);
        hold on;
        plot(t_0,Non_Linear_Hight,'-.g');
        hold off;
        title('Hight VS Time (Problem 1-3)');
        xlabel('Time');
        ylabel('Hight');
        legend('Linear Hight','Non Linaer Hight','Location','northwest')
%% Close Loop Transfer Function (Problem 2-1)        
[num_close , den_close] = feedback(-num_open , den_open , 1 , 1)
T_close = tf(num_close , den_close)
%% Depict Step Response Of Close Loop Transfer Function
T_approx=tf([0 1.29380197],[1 0.0222002 1.29380197])
 y1=step(T_close,t_0);
 y2=step(T_approx,t_0);
 figure,plot(t_0,y1);
        hold on;
        plot(t_0,y2,'-.g');
        hold off;
        title('Step Response Of Closed Loop And Approximate Close Loop (Problem 2-2)');
        xlabel('Time');
        ylabel('Step Response');
        legend('Closed Loop','Approximate Close Loop','Location','southwest')
        %% Problem 2-4 Plot Of Extra Zeros
 figure,plot(t_0,zero_10,'b');
        hold on;
        plot(t_0,zero_5,'-.g');
        plot(t_0,zero_1,'r');
        plot(t_0,zero_05,'k');
        plot(t_0,zero_0,'m');
        hold off;
        title('Effect Of Adding Extra Zeroes(Problem 2-4)');
        xlabel('Time');
        ylabel('Hight');
        legend('zero 10','zero 5','zero 1','zero 0.5','zero 0','Location','southeast')
         %% Problem 2-5 Plot Of Extra Poles
 figure,plot(t_0,Pole_10,'b');
        hold on;
        plot(t_0,Pole_5,'-.g');
        plot(t_0,Pole_1,'r');
        plot(t_0,Pole_05,'k');
        plot(t_0,Pole_0,'m');
        hold off;
        title('Effect Of Adding Extra Poles(Problem 2-5)');
        xlabel('Time');
        ylabel('Hight');
        legend('Pole 10','Pole 5','Pole 1','Pole 0.5','Pole 0','Location','northwest')
        %% Problem 3-1 Plot Of Variable Gain For Open Loop
 figure,plot(t_0,gain_1,'b');
        hold on;
        plot(t_0,gain_15,'-.g');
        plot(t_0,gain_05,'r');
        hold off;
        title('Effect Of Variable Gain For Open Loop (Problem 3-1)');
        xlabel('Time');
        ylabel('Hight');
        legend('Gain 1','Gain 1.5','Gain 0.5','Location','northwest')
        %% Problem 3-1 Obtain Of Close Loop Transfer Function For Variable Gain
        G_open_gain_15=tf([-1.94 -0.01123],[1 0.03088 -0.0002205 0]) %Open Loop Transfer Function With Gain=1.5
        G_open_gain_05=tf([-0.647 -0.005615],[1 0.03088 -0.0002205 0]) %Open Loop Transfer Function With Gain=0.5
        [num_close_gain_15 , den_close_gain_15] = feedback(-[-1.94 -0.01123] , [1 0.03088 -0.0002205 0] , 1 , 1)%Close Loop Transfer Function With Gain=1.5
        T_close_gain_15 = tf(num_close_gain_15 , den_close_gain_15)
        [num_close_gain_05 , den_close_gain_05] = feedback(-[-0.647 -0.005615] , [1 0.03088 -0.0002205 0] , 1 , 1)%Close Loop Transfer Function With Gain=0.5
        T_close_gain_05 = tf(num_close_gain_05 , den_close_gain_05)
        %% Problem 3-1 Plot Of Variable Gain For Close Loop
 figure,plot(t_0,gain_1_close,'b');
        hold on;
        plot(t_0,gain_15_close,'-.g');
        plot(t_0,gain_05_close,'r');
        hold off;
        title('Effect Of Variable Gain For Close Loop (Problem 3-1)');
        xlabel('Time');
        ylabel('Hight');
        legend('Gain 1','Gain 1.5','Gain 0.5','Location','northwest')
        %% Close Loop Transfer Function Variable Sensor Gain (Problem 3-2)        
[num_close_sensor_gain_1 , den_close_sensor_gain_1] = feedback(-num_open , den_open , 1 , 1)
T_close_sensor_gain_1 = tf(num_close_sensor_gain_1 , den_close_sensor_gain_1)
[num_close_sensor_gain_05 , den_close_sensor_gain_05] = feedback(-num_open , den_open , 1 , 2)
T_close_sensor_gain_05 = tf(num_close_sensor_gain_05 , den_close_sensor_gain_05)
[num_close_sensor_gain_2 , den_close_sensor_gain_2] = feedback(-num_open , den_open , 2 , 1)
T_close_sensor_gain_2 = tf(num_close_sensor_gain_2 , den_close_sensor_gain_2)
%% Problem 3-2 Plot Of Variable Sensor Gain For Close Loop (Problem 3-2)
 figure,plot(t_0,sensor_1_close,'b');
        hold on;
        plot(t_0,sensor_05_close,'-.g');
        plot(t_0,sensor_2_close,'r');
        hold off;
        title('Effect Of Variable Sensor Gain For Close Loop (Problem 3-2)');
        xlabel('Time');
        ylabel('Hight');
        legend('Sensor Gain 1','Sensor Gain 0.5','Sensor Gain 2','Location','northwest')