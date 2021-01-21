function ds = linear_aircraft_sim(~,s,vehicle)

% expand state deltas
du = s(4);
v = s(5);
w = s(6);
phi = s(7);
dtheta = s(8);
psi = s(9);
q = s(11);
p = s(10);
r = s(12);

% expand trim variables
theta0 = vehicle.trim.theta;
u0 = vehicle.trim.u;

% expand remaining parameters
Ix = vehicle.params.I(1,1);
Iy = vehicle.params.I(2,2);
Iz = vehicle.params.I(3,3);
Ipx = det(vehicle.params.I([1 3],[1 3]))/Iz;
Ipz = det(vehicle.params.I([1 3],[1 3]))/Ix;
Ipzx = vehicle.params.I(3,1)/det(vehicle.params.I([1 3],[1 3]));
S = vehicle.params.S;
c = vehicle.params.cbar;
b = vehicle.params.b;
m = vehicle.params.m;
g = vehicle.environment.g;
Q = 1/2*vehicle.environment.rho*u0;
coefs = vehicle.coefs;

% controller
[Lc, Mc, Nc, Xc, Yc, Zc] = vehicle.controller(s);

% partials of X
Xu = -2*Q*S*sin(theta0) * coefs.Cw0 + Q*S * coefs.Cxu;
Xw = Q*S * coefs.Cxa;

% partials of Z
Zu = -2*Q*S * coefs.Cw0 + Q*S * coefs.Czu;
Zw = Q*S * coefs.Cza;
Zq = Q*c*S/2 * coefs.Czq;
Zwdot = Q*c*S/(2*u0) * coefs.Czad;

% partils of M
Mu = Q*c*S * coefs.Cmu;
Mw = Q*c*S * coefs.Cma;
Mq = Q*c^2*S/2 * coefs.Cmq;
Mwdot = Q*c^2*S/(2*u0) * coefs.Cmad;

% longitudinal jacobian
Alon = [Xu/m Xw/m 0 -g*cos(theta0); Zu/(m-Zwdot) ...
    Zw/(m-Zwdot) (Zq+m*u0)/(m-Zwdot) -m*g*sin(theta0)/(m-Zwdot); ...
    (1/Iy)*(Mu + Mwdot*Zu/(m-Zwdot)) (1/Iy)*(Mw + Mwdot*Zw/(m-Zwdot)) ...
    (1/Iy)*(Mq + Mwdot*(Zq+m*u0)/(m-Zwdot)) -Mwdot*m*g*sin(theta0)/...
    (Iy*(m-Zwdot)); 0 0 1 0];

% longitudinal forcing
flon = [Xc/m; Zc/(m-Zwdot); Mc/Iy + Mwdot*Zc/(Iy*(m-Zwdot)); 0];

% partials of Y
Yv = Q*S * coefs.Cyb;
Yp = Q*b*S/2 * coefs.Cyp;
Yr = Q*b*S/2 * coefs.Cyr; 

% partials of L
Lv = Q*S*b * coefs.Clb;
Lp = Q*S*b^2/2 * coefs.Clp;
Lr = Q*S*b^2/2 * coefs.Clr;

% partils of N
Nv = Q*S*b * coefs.Cnb;
Np = Q*S*b^2/2 * coefs.Cnp;
Nr = Q*S*b^2/2 * coefs.Cnr;

% lateral jacobian
Alat = [Yv/m Yp/m Yr/m-u0 g*cos(theta0); Lv/Ipx+Ipzx*Nv Lp/Ipx+Ipzx*Np ...
    Lr/Ipx+Ipzx*Nr 0; Ipx*Lv+Nv/Ipz Ipzx*Lp+Np/Ipz Ipzx*Lr+Nr/Ipz 0; ...
    0 1 tan(theta0) 0];

% lateral forcing
flat = [Yc/m; Lc/Ipx+Ipzx*Nc; Ipzx*Lc+Nc/Ipz; 0];

% compute the longitudinal derivatives (eq 4.9,18)
ds([4 6 11 8]) = Alon*[du; w; q; dtheta] + flon;
ds(1) = du*cos(theta0) + w*sin(theta0) - u0*dtheta*sin(theta0);
ds(3) = -du*sin(theta0) + w*cos(theta0) - u0*dtheta*cos(theta0);

% compute the lateral derivatives (eq 4.9,19)
ds([5 10 12 7]) = Alat*[v; p; r; phi] + flat;
ds(9) = r*sec(theta0);
ds(2) = u0*psi*cos(theta0)+v;

% return a colunm vector
ds = ds(:);

end