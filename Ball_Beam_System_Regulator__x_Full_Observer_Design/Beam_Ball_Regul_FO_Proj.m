function DX=Beam_Ball_Regul_FO_Proj(t,X,u)

global A B m g J_Beam

A = [0 1 0 0;0 0 (-g*sin(X(3)))/(1.4*X(3)) (X(1)*X(4))/(1.4);0 0 0 1;(m*g*cos(X(3)))/(m*X(1)^2+J_Beam) 0 0  (-2*m*X(1)*X(2))/(m*X(1)^2+J_Beam)];
B = [0;0;0;1/(m*X(1)^2+J_Beam)];

DX= A*X + B*u;