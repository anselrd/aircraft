function dx = simple_quad_sim(~,x,params)
%Computes the time rates of change of the state vector of a quadcopter for
% use in ode45
% Inputs: t - Time
%         x - State vector [x_E y_E z_E u v w phi theta psi p q r]' with
%                           positions in meters, velocities in meters per
%                           second, euler angles in degrees, and p, q, and 
%                           r in radians per second
%         m - Mass of the quadcopter
%         I - Body frame inertia matrix of the quadcopter
%         thruster_locs - A 4-by-3 matrix where each row is the body frame
%                         coordinates of a thruster

% expand inputs
u = x(4);
v = x(5);
w = x(6);
phi = deg2rad(x(7));
theta = deg2rad(x(8));
psi = deg2rad(x(9));
p = x(10);
q = x(11);
r = x(12);
Ix = params.I(1,1);
Iy = params.I(2,2);
Iz = params.I(3,3);

% define physical parameters
g = 9.81;

% set trim thrusts
current_controls = (params.F/4)*ones(1,4);

% set the wind
Wx = 0;
Wy = 0;
Wz = 0;

% compute the resultant external force from controls
Xc = 0;
Yc = 0;
Zc = -sum(current_controls);

% compute the resultant external force form aerodynamics
V = [u v w]';
Xa = -params.eta*u^2*sign(u);
Ya = -params.eta*v^2*sign(v);
Za = -params.zeta*w^2*sign(w);

% compute the net external force
X = Xa + Xc;
Y = Ya + Yc;
Z = Za + Zc;

% find inertial velocity
uE = u + Wx;
vE = v + Wy;
wE = w + Wz;

% compute inertial position derivatives
xdotE = uE*cos(theta)*cos(psi) + vE*(sin(phi)*sin(theta)*cos(psi) - ...
    cos(phi)*sin(psi)) + wE*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
ydotE = uE*cos(theta)*sin(psi) + vE*(sin(phi)*sin(theta)*sin(psi) + ...
    cos(phi)*cos(psi)) + wE*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
zdotE = -uE*sin(theta) + vE*sin(phi)*cos(theta) + wE*cos(phi)*cos(theta);

% compute acceleration (constant wind, udot = udotE)
udot = (X - params.m*g*sin(theta))/params.m - q*wE + r*vE;
vdot = (Y + params.m*g*cos(theta)*sin(phi))/params.m - r*uE + p*wE;
wdot = (Z + params.m*g*cos(theta)*cos(phi))/params.m - p*vE + q*uE;

% compute derivatives of the euler angles
phidot = rad2deg(p + (q*sin(phi) + r*cos(phi))*tan(theta));
thetadot = rad2deg(q*cos(phi) - r*sin(phi));
psidot = rad2deg((q*sin(phi) + r*cos(phi))*sec(theta));

% compute conrol moments
G_Bc = [0 0 0]';
for k = 1:4
    G_Bc = G_Bc + ...
        cross(params.thruster_locs(k,1:3)',[0 0 -current_controls(k)])';
end
Lc = G_Bc(1);
Mc = G_Bc(2);
Nc = G_Bc(3) - params.k*(params.thruster_locs(:,4)'*current_controls');

% compute aerodynamic moments
La = -params.alpha*p^2*sign(p);
Ma = -params.alpha*q^2*sign(q);
Na = -params.beta*r^2*sign(r);

% compute total external moment
L = La + Lc;
M = Ma + Mc;
N = Na + Nc;

% compute derivatives of angular velocity
pdot = (1/Ix)*(L - q*r*(Iz-Iy));
qdot = (1/Iy)*(M - r*p*(Ix-Iz));
rdot = (1/Iz)*(N - p*q*(Iy-Ix));

% assign to angular derivatives
dx = [xdotE  ydotE    zdotE  ...
      udot   vdot     wdot   ...
      phidot thetadot psidot ...
      pdot   qdot     rdot]';

end