function DX=MIMO_DOuble_Pendulum_Regul_Proj(t,X,u)

DX1 = X(2);
DX2 = (140*(cos(X(5)) - cos(X(3))*cos(X(3) - X(5)))*((sin(X(3) - X(5))*X(4)^2)/20 + sin(X(5))))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119) + (20*(7*cos(X(3)) - 2*cos(X(5))*cos(X(3) - X(5)))*(u(2) + (7*sin(X(3)))/2 - (X(6)^2*sin(X(3) - X(5)))/20))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119) + (10*(2*cos(X(3) - X(5))^2 - 7)*((7*sin(X(3))*X(4)^2)/20 + (sin(X(5))*X(6)^2)/10 + u(1)))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119);
DX3 = X(4);
DX4 = (40*((sin(X(3) - X(5))*X(4)^2)/20 + sin(X(5)))*(17*cos(X(3) - X(5)) - 7*cos(X(3))*cos(X(5))))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119) + (40*(2*cos(X(5))^2 - 17)*(u(2) + (7*sin(X(3)))/2 - (X(6)^2*sin(X(3) - X(5)))/20))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119) + (20*(7*cos(X(3)) - 2*cos(X(5))*cos(X(3) - X(5)))*((7*sin(X(3))*X(4)^2)/20 + (sin(X(5))*X(6)^2)/10 + u(1)))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119);
DX5 = X(6);
DX6 = (40*(17*cos(X(3) - X(5)) - 7*cos(X(3))*cos(X(5)))*(u(2) + (7*sin(X(3)))/2 - (X(6)^2*sin(X(3) - X(5)))/20))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119) + (140*(cos(X(5)) - cos(X(3))*cos(X(3) - X(5)))*((7*sin(X(3))*X(4)^2)/20 + (sin(X(5))*X(6)^2)/10 + u(1)))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119) + (140*((sin(X(3) - X(5))*X(4)^2)/20 + sin(X(5)))*(7*cos(X(3))^2 - 17))/(49*cos(X(3))^2 + 14*cos(X(5))^2 + 34*cos(X(3) - X(5))^2 - 28*cos(X(3))*cos(X(5))*cos(X(3) - X(5)) - 119);
DX= [DX1;DX2;DX3;DX4;DX5;DX6];