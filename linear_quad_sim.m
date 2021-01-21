function ds = linear_quad_sim(~,s,vehicle)

% expand state deltas
du = s(4);
w = s(6);
q = s(11);
dtheta = s(8);
v = s(5);
p = s(10);
r = s(12);
phi = s(7);
psi = s(9);

% expand trim variables
theta0 = deg2rad(vehicle.trim.theta);
u0 = vehicle.trim.u;
v0 = vehicle.trim.v;
w0 = vehicle.trim.w;
p0 = vehicle.trim.p;
q0 = vehicle.trim.q;
r0 = vehicle.trim.r;

% expand remaining parameters
Ix = vehicle.params.I(1,1);
Iy = vehicle.params.I(2,2);
Iz = vehicle.params.I(3,3);
Ipx = det(vehicle.params.I([1 3],[1 3]))/Iz;
Ipz = det(vehicle.params.I([1 3],[1 3]))/Ix;
Ipzx = vehicle.params.I(3,1)/det(vehicle.params.I([1 3],[1 3]));
eta = vehicle.params.eta;
zeta = vehicle.params.zeta;
alpha = vehicle.params.alpha;
beta = vehicle.params.beta;
m = vehicle.params.m;
g = vehicle.environment.g;

%Controller
[Lc, Mc, Nc, Xc, Yc, Zc] = vehicle.controller(s);

% partials of X
Xu = -2*eta*u0*sign(u0);
Xw = 0;

% partials of Z
Zu = 0;
Zw = -2*zeta*w0*sign(w0);
Zq = 0;
Zwdot = 0;

% partils of M
Mu = 0;
Mw = 0;
Mq = -2*alpha*q0*sign(q0);
Mwdot = 0;

% longitudinal jacobian
lon = [Xu/m Xw/m 0 -g*cos(theta0); Zu/(m-Zwdot) ...
    Zw/(m-Zwdot) (Zq+m*u0)/(m-Zwdot) -m*g*sin(theta0)/(m-Zwdot); ...
    (1/Iy)*(Mu + Mwdot*Zu/(m-Zwdot)) (1/Iy)*(Mw + Mwdot*Zw/(m-Zwdot)) ...
    (1/Iy)*(Mq + Mwdot*(Zq+m*u0)/(m-Zwdot)) -Mwdot*m*g*sin(theta0)/...
    (Iy*(m-Zwdot)); 0 0 1 0];

% longitudinal forcing
flon = [Xc/m; Zc/(m-Zwdot); Mc/Iy + Mwdot*Zc/(Iy*(m-Zwdot)); 0];

% partials of Y
Yv = -2*eta*v0*sign(v0);
Yp = 0;
Yr = 0; 

% partials of L
Lv = 0;
Lp = -2*alpha*p0*sign(p0);
Lr = 0;

% partils of N
Nv = 0;
Np = 0;
Nr = -2*beta*r0*sign(r0);

% lateral jacobian
lat = [Yv/m Yp/m Yr/m-u0 g*cos(theta0); Lv/Ipx+Ipzx*Nv Lp/Ipx+Ipzx*Np ...
    Lr/Ipx+Ipzx*Nr 0; Ipx*Lv+Nv/Ipz Ipzx*Lp+Np/Ipz Ipzx*Lr+Nr/Ipz 0; ...
    0 1 tan(theta0) 0];

% lateral forcing
flat = [Yc/m; Lc/Ipx+Ipzx*Nc; Ipzx*Lc+Nc/Ipz; 0];

% compute the longitudinal derivatives (eq 4.9,18)
ds([4 6 11 8]) = lon*[du; w; q; dtheta] + flon;
ds(1) = du*cos(theta0) + w*sin(theta0) - u0*dtheta*sin(theta0);
ds(3) = -du*sin(theta0) + w*cos(theta0) - u0*dtheta*cos(theta0);

% compute the lateral derivatives (eq 4.9,19)
ds([5 10 12 7]) = lat*[v; p; r; phi] + flat;
ds(9) = r*sec(theta0);
ds(2) = u0*psi*cos(theta0)+v;

% return a colunm vector
ds = ds(:);

end

