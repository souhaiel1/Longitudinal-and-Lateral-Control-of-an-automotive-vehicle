function xdot = CarDynamics(Vx,w,Theta,Dry)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
m=1400 ; %Kg
R= 0.3 ; %m
Cx= 66100;  %N
Fx=longForce(m,R,w,Vx,Dry) ;
Fd=longDrag(m,Vx,Theta);
acc=Fx-Fd/m;
xdot= acc;
end

%function Fx = longForce(Cx,R,w,Vx) 
function Fx = longForce(m,R,w,Vx,Dry)
if Dry
    v=[1.11 23.99 0.52];
else 
    v=[ 0.687 33.822 0.347];
end
if R*w>=Vx 
    slip = (R*w-Vx)/(R*w) ;
else 
    slip=(R*w-Vx)/Vx;
end 
%Buck Hart model 
%mu=v(1)*(1-exp(-v(2)*slip))-v(3)*slip ; 
mu= sign(slip)*(v(1)*(1-exp(-v(2)*abs(slip)))-v(3)*abs(slip)) ;
g=9.81 ;
Fx= mu*m*g/2 ; % mu x N
%Fx=Cx * slip ; 
end
function Fd=longDrag(m,Vx,Theta)
A=0.14 ;
Cd=0.35;
Cr=0.015; 
g=9.81;
rho=1.225;
Fd=0.5*rho*Cd*A*(Vx^2)+Cr*m*g*cos(Theta)+m*g*sin(Theta);
end

