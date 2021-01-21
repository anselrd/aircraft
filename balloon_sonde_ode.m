function [ dx ] = balloon_sonde_ode(~,y,m,wind,r,Cd)

g = 9.81;
rho = 1.225; % sea level
A = pi*r^2;
V = (4/3)*pi*r^3;

vrel = y(4:6)-wind;
magv = norm(vrel);
if magv == 0
    magv = 1;
end

buoy = rho*V*g/m;

dx(1:3) = y(4:6);
dx(4:6) = [0 0 g-buoy]' - rho*A*Cd*(vrel'*vrel)/(2*m) * vrel/magv;
dx = dx(:);

end