%% Secant_Method_Guidance_and_Navigation_HW#4

clear all 
clc

Long0 = 25; %degree%
Theta = Long0*pi/180;
Longf = 50; %degree%
tfd = 600; %s%
a = 6371000; %m%
R0 = 6371000; %m%
Rf = 6371000; %m%
GM = 3.986004418*(10^14); %m^3/s^2%
Phi = (Longf - Long0)*(pi/180);%rad%
error = 0.00001;

Gamma_min = atan(sin(Phi) - (sqrt((2*R0/Rf)*(1-cos(Phi))))/(1-cos(Phi)));%rad%
Gamma_max = atan(sin(Phi) + (sqrt((2*R0/Rf)*(1-cos(Phi))))/(1-cos(Phi)));%rad%
Gamma0 = ((Gamma_max + Gamma_min)/2);%rad%
V0x = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(Gamma0))*((R0*cos(Gamma0))/(a) - cos(Phi+Gamma0))) )*sin(Gamma0)*cos(Theta);%m/s%
V0y = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(Gamma0))*((R0*cos(Gamma0))/(a) - cos(Phi+Gamma0))) )*sin(Gamma0)*sin(Theta);
V0 = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(Gamma0))*((R0*cos(Gamma0))/(a) - cos(Phi+Gamma0))) );
landa0 = (R0*V0^2)/GM;
tf0 = ( (R0)/(V0*cos(Gamma0)) )*( ( tan(Gamma0)*(1-cos(Phi)) + (1-landa0)*sin(Phi) )/( (2-landa0)*( (1-cos(Phi))/(landa0*cos(Gamma0)^2) + (cos(Gamma0 + Phi))/(cos(Gamma0)) ) ) + ( (2*cos(Gamma0))/(landa0*((2/landa0) - 1)^1.5) )*atan((sqrt((2/landa0) - 1)/((cos(Gamma0)*cot(Phi/2)) - (sin(Gamma0)))) ));

Gamma00 = 0;
tf00 = 0;
V00x = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(Gamma00))*((R0*cos(Gamma00))/(a) - cos(Phi+Gamma00))) )*sin(Gamma00)*cos(Theta);%m/s%
V00y = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(Gamma00))*((R0*cos(Gamma00))/(a) - cos(Phi+Gamma00))) )*sin(Gamma00)*sin(Theta);%m/s%
V00 = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(Gamma00))*((R0*cos(Gamma00))/(a) - cos(Phi+Gamma00))) );


Gamma = [];
tf = [];
Vr = [];
Vrx = [];
Vry = [];
Gamma(1) = Gamma00;
tf(1) = tf00;
Vr(1) = V00;
Vrx(1) = V00x;
Vry(1) = V00y;
Gamma(2) = Gamma0;
tf(2) = tf0;
Vr(2) = V0;
Vrx(1) = V0x;
Vry(1) = V0y;
i = 2;

%% Secant Algorithm
while abs(tfd - tf(i)) >= error
    
    Gamma(i+1) = Gamma(i) + ( (Gamma(i) - Gamma(i-1))*(tfd - tf(i)) )/(tf(i) - tf(i-1));
    
    gamma = Gamma(i+1);
    Vrx(i+1) = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(gamma))*((R0*cos(gamma))/(a) - cos(Phi+gamma))) )*sin(gamma)*cos(Theta);
    Vry(i+1) = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(gamma))*((R0*cos(gamma))/(a) - cos(Phi+gamma))) )*sin(gamma)*sin(Theta);
    Vr(i+1) = sqrt( (GM*( 1-cos(Phi)))/((R0*cos(gamma))*((R0*cos(gamma))/(a) - cos(Phi+gamma))) );
    vr = Vr(i+1);
    landa = (R0*vr^2)/GM;
    tf(i+1) = ( (R0)/(vr*cos(gamma)) )*( ( tan(gamma)*(1-cos(Phi)) + (1-landa)*sin(Phi) )/( (2-landa)*( (1-cos(Phi))/(landa*cos(gamma)^2) + (cos(gamma + Phi))/(cos(gamma)) ) ) + ( (2*cos(gamma))/(landa*((2/landa) - 1)^1.5) )*atan2((sqrt((2/landa) - 1)),((cos(gamma)*cot(Phi/2)) - (sin(gamma)))) );

    
    i = i+1;
    
end

%% Display
fprintf('The Initial Flight Path Angle Must be (deg)\n')
disp(Gamma(end)*180/pi)

fprintf('The Initial X-Velocity Must be (m/s)\n')
disp(Vrx(end))

fprintf('The Initial Y-Velocity Must be (m/s)\n')
disp(Vry(end))

fprintf('The Initial Velocity Must be (m/s)\n')
disp(Vr(end))

fprintf('Flight Time\n')
disp(tf(end))