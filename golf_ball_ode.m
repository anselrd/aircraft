function [ dx ] = golf_ball_ode(~,y,m,wind,d,Cd)

g = 9.81;
rho = 1.225; % sea level
A = pi*(d/2)^2;

vrel = y(4:6)-wind;

dx(1:3) = y(4:6);
dx(4:6) = [0 0 g]' - rho*A*Cd*(vrel'*vrel)/(2*m) * vrel/norm(vrel);
dx = dx(:);

end