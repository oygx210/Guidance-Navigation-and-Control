function DXh=Pendulum_Full_Obser_Regual_Proj(t,Xh,u,y)
global  A B C_Yx Ko_Yx


yh = C_Yx*Xh;
DXh = A*Xh + B*u + Ko_Yx*(y-yh);