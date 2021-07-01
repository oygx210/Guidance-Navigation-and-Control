function DX=Pendulum_Servo_FF_MO_Obs_Proj(t,X,u)

global  A B m M_Cart g l Aaa Aab Abb Aba Ba Bb

A = [0 1 0 0;0 0 (-m*g*sin(X(3))*cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)) (m*l*X(4)*sin(X(3)))/(M_Cart+m*(sin(X(3)))^2);0 0 0 1;0 0 ((M_Cart+m)*g*sin(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)*l) (-m*l*X(4)*sin(X(3)*cos(X(3))))/((M_Cart+m*(sin(X(3)))^2)*l)];
B = [0;1/(M_Cart+m*(sin(X(3)))^2);0;(-cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*l)];
Aaa = A(1);
Aab = A(1,2:4);
Aba = A(2:4,1);
Abb = [A(2,2:4);A(3,2:4);A(4,2:4)];
Ba = B(1);
Bb = B(2:4,1);


DX= A*X + B*u;