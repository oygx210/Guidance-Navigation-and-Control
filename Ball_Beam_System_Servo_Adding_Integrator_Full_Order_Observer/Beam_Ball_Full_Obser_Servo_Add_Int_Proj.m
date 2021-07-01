function DXh=Beam_Ball_Full_Obser_Servo_Add_Int_Proj(t,Xh,u,y)
global Ko_Yx A B C_Yx

yh = C_Yx*Xh;
DXh = A*Xh + B*u + Ko_Yx*(y-yh);