function DX=Pendulum_LQR(t,X,u)

global M_Cart m g l

A = [0 1 0 0;0 0 (-m*g*sin(X(3))*cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)) (m*l*X(4)*sin(X(3)))/(M_Cart+m*(sin(X(3)))^2);0 0 0 1;0 0 ((M_Cart+m)*g*sin(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)*l) (-m*l*X(4)*sin(X(3)*cos(X(3))))/((M_Cart+m*(sin(X(3)))^2)*l)];
B = [0;1/(M_Cart+m*(sin(X(3)))^2);0;(-cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*l)];

DX=A*X+B*u;