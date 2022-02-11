function xdot = LongDynWorking(Vx,YawRate, Beta,w,delta,Theta,Dry)

% w long 
% delta lat 
%Paramaters
m = 1400; %kg
R = 0.3;  %m
Cx = 66100; %N
lr = 1.3580 ; 
lf= 1.1770 ;
Iz= 1960 ; 
Fx = longForce(m,R,w,Vx,Dry);
Fd = longDrag(m,Vx,Theta);
acc = (Fx-Fd)/m+Beta*Vx*YawRate;

Fyf= 2*lateralForeFront(Vx,YawRate,Beta,w,delta,R,lf,Dry)  ; 
Fyr=2*latForceRear(Vx,YawRate,Beta,lr,Dry) ; 
% Longitidunal forces
Fx = longForce(m,R,w,Vx,Dry) ; 
Fd= longDrag(m,Vx,Theta) ; 

% Lateral forces 
YawRate_dot= (Fyf*lf-Fyr*lr)/Iz;
Beta_dot= (Fyf+Fyr)/(m*Vx)-YawRate ;
xdot=[acc;YawRate_dot;Beta_dot];
end

function Fx = longForce(m,R,w,Vx,Dry)

if Dry
    v = [1.11 23.99 0.52];
else
    v = [0.687 33.822 0.347];
end

if R*w>=Vx %if accelerating
    slip = (R*w-Vx)/(R*w);
else
    slip = (R*w-Vx)/Vx;
end

%Buck-Hart Model
mu = sign(slip)*(v(1)*(1-exp(-v(2)*abs(slip)))-v(3)*abs(slip));

g = 9.81;
Fx = mu*m*g/2; %mu*N

end



% alpha is theta = delta - alphaf
% slip 
function Fy = tirelateralPlacejkmodel(alpha,slip,Dry)
if Dry 
    mu =1; 
else 
    mu=.5;
end
B= 2*(2-mu)*8.3278 ; 
C = (5/4-mu/4)*1.1009 ; 
D=mu*2268 ; 
E= -1.1661 ; 
Fy = D*sin(C*atan(B*(1-E)*alpha+E*atan(B*alpha)))*exp(-6*(abs(slip))^5) ;

end
 

function Fyf = lateralForeFront(Vx,YawRate,Beta,w,delta,R,lf,Dry) 

if R*w>=Vx % if accelerating
    slip = (R*w-Vx)/(R*w) ; 
else 
    slip = (R*w-Vx)/(Vx) ;
end 
alpha_f = delta - atan ((Vx*Beta+lf*YawRate)/Vx) ;
Fyf= tirelateralPlacejkmodel(alpha_f, slip,Dry); 

end


function Fyr = latForceRear(Vx,YawRate,Beta,lr,Dry) 
alpha_r=-atan((Vx*Beta-lr*YawRate)/Vx); 
Fyr= tirelateralPlacejkmodel(alpha_r, 0, Dry) ; 

end 



function Fd = longDrag(m,Vx,Theta)
rho = 1.225;
Cd = 0.35;
A = 2.14;
Cr = 0.015;
g=9.81;

Fd = 0.5*rho*Cd*A*Vx^2+Cr*m*g*cos(Theta)+m*g*sin(Theta);

% Cr*m*g*cos(Theta) : friction in rear wheel 
end
