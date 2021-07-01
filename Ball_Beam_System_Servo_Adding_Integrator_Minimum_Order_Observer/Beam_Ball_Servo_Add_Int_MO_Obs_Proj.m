function DX=Beam_Ball_Servo_Add_Int_MO_Obs_Proj(t,X,u,yr)
global C_Yx A B Aaa Aab Aba Abb Ba Bb g m J_Beam

A = [0 1 0 0;0 0 (-g*sin(X(3)))/(1.4*X(3)) (X(1)*X(4))/(1.4);0 0 0 1;(m*g*cos(X(3)))/(m*X(1)^2+J_Beam) 0 0  (-2*m*X(1)*X(2))/(m*X(1)^2+J_Beam)];
B = [0;0;0;1/(m*X(1)^2+J_Beam)];
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