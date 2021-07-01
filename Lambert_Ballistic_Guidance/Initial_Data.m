%% Initial_Data_Guidance_and_Navigation_HW#4

Long0 = 35; %degree%
Longf = 50; %degree%
a = 6371000; %m%
R0 = 6371000; %m%
Rf = 6371000; %m%
GM = 3.986004418*(10^14); %m^3/s^2%
Phi = (Longf - Long0)*(pi/180);%rad%
Gamma_min = atan(sin(Phi) - (sqrt((2*R0/Rf)*(1-cos(Phi))))/(1-cos(Phi)));%rad%
Gamma_max = atan(sin(Phi) + (sqrt((2*R0/Rf)*(1-cos(Phi))))/(1-cos(Phi)));%rad%
Gamma0 = 60*pi/180;%rad%
V0 = 10;%m/s%
errorV = 1;