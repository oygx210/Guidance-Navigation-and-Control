function DEth=Pendulum_Min_Obser_Servo_FF_Proj(t,Eth,u,y)
global Aaa Aab Aba Abb Ba Bb Ko_Yx

DEth=Abb*Eth + Abb*Ko_Yx*y + Aba*y + Bb*u + Ko_Yx*(-Aaa*y - Aab*Eth - Aab*Ko_Yx*y - Ba*u);