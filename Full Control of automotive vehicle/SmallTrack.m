load Points.mat

theta = -45*pi/180;

R = [cos(theta) -sin(theta);sin(theta) cos(theta)];

Data001 = Data001*R;

X = Data001(:,1);
Y = Data001(:,2);

plot(X,Y)
axis equal


%% Spline Sections
dx = 0.01;

%section 1
x = X(1:31);
y = Y(1:31);
xs1 = flip(x(end):dx:x(1));
s1 = pchip(x,y,xs1);

%section 2
x = X(31:66);
y = Y(31:66);
xs2 = x(1):dx:x(end);
s2 = pchip(x,y,xs2);

%section 3
x = X(66:end);
y = Y(66:end);
xs3 = flip(x(end):dx:x(1));
s3 = pchip(x,y,xs3);

xs = [xs1,xs2(2:end),xs3(2:end)];
s = [s1,s2(2:end),s3(2:end)];

hold on
plot(xs,s)


%%

theta = 45*pi/180;
R = [cos(theta) -sin(theta);sin(theta) cos(theta)];

Data = [xs',s'];
Data = Data*R;

X = Data(:,1);
Y = Data(:,2);

figure
plot(X,Y)
axis equal

%%
Track.X = X;
Track.Y = Y;