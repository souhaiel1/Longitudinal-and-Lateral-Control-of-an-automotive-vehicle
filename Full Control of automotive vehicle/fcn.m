function y=fcn(Psi, Vx Beta) 

Vy=Bet*Vy ; 
V=sqrt(Vx^2+Vy^2); 

Xdot=Vcos(Psi+Beta); 
Ydot=V*sin(Psi+Beta) ; 

y=[Xdot;Ydot] ; 
end 


