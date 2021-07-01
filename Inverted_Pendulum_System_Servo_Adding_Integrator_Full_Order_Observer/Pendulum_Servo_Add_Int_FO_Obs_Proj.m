function DX=Pendulum_Servo_Add_Int_FO_Obs_Proj(t,X,u,yr)
global  C_Yx A B m g M_Cart l

A = [0 1 0 0;0 0 (-m*g*sin(X(3))*cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)) (m*l*X(4)*sin(X(3)))/(M_Cart+m*(sin(X(3)))^2);0 0 0 1;0 0 ((M_Cart+m)*g*sin(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)*l) (-m*l*X(4)*sin(X(3)*cos(X(3))))/((M_Cart+m*(sin(X(3)))^2)*l)];
B = [0;1/(M_Cart+m*(sin(X(3)))^2);0;(-cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*l)];

Xx = X(1:4);
Xi = X(5);
y = C_Yx*Xx;

DXx = A*Xx + B*u;
DXi = yr - y;
DX = [DXx;DXi];