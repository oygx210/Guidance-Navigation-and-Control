function DXh=Beam_Ball_Phi_Full_Obser_Regual_Proj(t,Xh,u,y)
global A B C_Yphi Ko_Yphi

yh = C_Yphi*Xh;
DXh = A*Xh + B*u + Ko_Yphi*(y-yh);