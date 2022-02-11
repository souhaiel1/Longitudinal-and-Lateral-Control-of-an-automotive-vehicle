%% Enter Controllers parameters

% PI Parameters
Kp = 1500;     % Proportional gain Kp
% Tl = ;    % Integral gain Ki = Kp*Tl
Ki=200; 
ts = 0.1;    % Closed Loop settling time (For the antiwindup)
Fmax = 9000;  % Maximum longitudinal tire force (Used for the saturation of the PI)
Ka=1/(Ki*ts/3);
% MPC data ( To be Entered )
Q =10 ;     %Weight on the tracking error
R = 2;     %Weight on the control action  
N = 5;     %Prediction Horizon

MPC_Script
Montmelo

%% Initial Conditions (this parts needs to be executed before every simulation

global pathindex
pathindex = 1;

X0 = 0;
Y0 = 0;
Psi0 = 245*pi/180;
V0 = 20;

%%
%%% AFTER running the simulation you can use this section to compare the trajectory of the car
%%% with the desired trajectory
figure(2)
plot(Track.X,Track.Y)
hold on
plot(out.Xgl.Data(1,:),out.Ygl.Data(1,:)) 
legend('track reference','simulated path');
axis equal
