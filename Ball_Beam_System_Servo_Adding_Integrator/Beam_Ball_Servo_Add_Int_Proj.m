function DX=Beam_Ball_Servo_Add_Int_Proj(t,X,u,yr)
global J_Beam m g C_Yx A B

A = [0 1 0 0;0 0 (-g*sin(X(3)))/(1.4*X(3)) (X(1)*X(4))/(1.4);0 0 0 1;(m*g*cos(X(3)))/(m*X(1)^2+J_Beam) 0 0  (-2*m*X(1)*X(2))/(m*X(1)^2+J_Beam)];
B = [0;0;0;1/(m*X(1)^2+J_Beam)];

Xx = X(1:4);
Xi = X(5);
y = C_Yx*Xx;

DXx = A*Xx + B*u;
DXi = yr - y;
DX = [DXx;DXi];