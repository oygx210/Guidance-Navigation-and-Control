function DX=Pendulum_Servo_Add_Int_MO_Obs_Proj(t,X,u,yr)
global C_Yx A B Aaa Aab Aba Abb Ba Bb l m g M_Cart


A = [0 1 0 0;0 0 (-m*g*sin(X(3))*cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)) (m*l*X(4)*sin(X(3)))/(M_Cart+m*(sin(X(3)))^2);0 0 0 1;0 0 ((M_Cart+m)*g*sin(X(3)))/((M_Cart+m*(sin(X(3)))^2)*X(3)*l) (-m*l*X(4)*sin(X(3)*cos(X(3))))/((M_Cart+m*(sin(X(3)))^2)*l)];
B = [0;1/(M_Cart+m*(sin(X(3)))^2);0;(-cos(X(3)))/((M_Cart+m*(sin(X(3)))^2)*l)];
Aaa = A(1);
Aab = A(1,2:4);
Aba = A(2:4,1);
Abb = [A(2,2:4);A(3,2:4);A(4,2:4)];
Ba = B(1);
Bb = B(2:4,1);

Xx = X(1:4);
Xi = X(5);
y = C_Yx*Xx;

DXx = A*Xx + B*u;
DXi = yr - y;
DX = [DXx;DXi];