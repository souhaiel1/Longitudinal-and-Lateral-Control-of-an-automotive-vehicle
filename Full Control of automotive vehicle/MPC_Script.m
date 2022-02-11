%% Paramaters
m = 1400;
Iz = 1960;
lf = 1.1770;
lr = 1.3580;
Cf = 84085;
Cr = 87342;
g = 9.8100;

Ts = 0.01;

vx = 25;

%% Matrix Model

A = [-(lf^2*Cf+lr^2*Cr)/(Iz*vx) (-lf*Cf+lr*Cr)/(Iz);
     -1+(-lf*Cf+lr*Cr)/(m*vx^2) -(Cf+Cr)/(m*vx)];
B = [lf*Cf/Iz; Cf/(m*vx)];

% Discretize for the MPC

%Euler discretization
Ad = eye(2)+Ts*A;       %% Euler discretization Ad = (I+Ts*A)
Bd = Ts*B;              %% Euler discretization Bd = Ts*B
%Exact discretization
sysd = c2d(ss(A,B,eye(2),0),Ts);
Ad = sysd.A;  
Bd = sysd.B;         

nx = 2; % Number of states
nu = 1; % Number of inputs

%% MPC data ( To be Entered )
% Q = 40; %weight on state
% R = 3; %weight on input 
% N =30; %prediction horizon

C = [1 0]; %% For reference tracking, only the first state is used

% MPC Formulation

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
r = sdpvar(1,1);
Ad_ = sdpvar(2,2,'full'); 
Bd_ = sdpvar(2,1);
constraints = [];
objective = 0;
for k = 1:N
 objective = objective + (r-C*x{k})'*Q*(r-C*x{k}) + u{k}*R*u{k};

 %Constraint on Rate of Variation of the input u (Check the website, but
 %its optional)
 
 
 constraints = [constraints, x{k+1} == Ad*x{k} + Bd*u{k}];
 constraints = [constraints, -60*pi/180 <= u{k}<= 60*pi/180];
end

%controller = optimizer(constraints, objective,[],{x{1},r},[u{:}]);
controller = optimizer(constraints, objective,[],{x{1},r,Ad_,Bd_},[u{:}]);

%% Test MPC

x0 = [0;0];
ref = 0.1;
sysd = c2d(ss(A,B,eye(2),0),Ts);
Ad_ = sysd.A;  
Bd_ = sysd.B;
controller(x0,ref,Ad_,Bd_)

MPC_Sim(vx,x0(1),x0(2),ref,controller)






