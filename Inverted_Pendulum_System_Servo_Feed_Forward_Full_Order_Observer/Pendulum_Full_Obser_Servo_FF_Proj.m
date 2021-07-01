function DXh=Pendulum_Full_Obser_Servo_FF_Proj(t,Xh,u,y)
global Ko_Yx A B C_Yx


yh = C_Yx*Xh;
DXh = A*Xh + B*u + Ko_Yx*(y-yh);