function [dx, u] = nonlinear_quad_sim(~,x,vehicle)
%Computes the time rates of change of the state vector of a quadcopter for
% use in ode45
% Inputs: t - Time
%         x - State vector [x_E y_E z_E u v w phi theta psi p q r]' with
%                           positions in meters, velocities in meters per
%                           second, euler angles in degrees, and p, q, and 
%                           r in radians per second
%         vehicle - struct which contains the relevant information about
%                   vehicle being simulated.
%                   field    subfield         | description
%                   ------------------------------------------------------
%                   params                    |
%                            m                | Vehicle mass [kg]
%                                             |
%                            d                | Quadcopter arm length [m]
%                                             |
%                            I                | Inertia matrix [kg m^2]
%                                             |
%                            thruster_locs    | 4-by-4 matrix. Each row is
%                                             | a 1-by-3 thruster position
%                                             | expressed in body frame
%                                             | coordinates, and the fourth
%                                             | entry is +/- 1 indicating
%                                             | rotor spin direction,
%                                             | corresponding to CW and CCW
%                                             | respectively.
%                                             |
%                            k                | Proportionality constant
%                                             | mapping thrust forces to
%                                             | yawing moments as defined
%                                             | in class
%                                             |
%                            eta              | Lateral translational drag
%                                             | coefficient defined in
%                                             | class
%                                             |
%                            zeta             | Vertical translational drag
%                                             | coefficient defined in
%                                             | class
%                                             |
%                            alpha            | Lateral rotational drag
%                                             | coefficient defined in
%                                             | class
%                                             |
%                            beta             | Vertical rotational drag
%                                             | coefficient defined in
%                                             | class
%                                             |
%                   environment               |
%                            g                | gravitational acceleration
%                                             | [m/s^2]
%                                             |
%                            Wx               | body x wind component [m/s]
%                                             |
%                            Wy               | body y wind component [m/s]
%                                             | 
%                            Wz               | body z wind component [m/s]
%                   controller                | a function which takes the
%                                             | vehicle state as an
%                                             | argument and returns the 
%                                             | control forces and moments
%                                             | Lc Mc Nc Xc Yc Zc along 
%                                             | with a vector of the
%                                             | control inputs u
% Outputs: dx - time derivatives of the states
%          u  - vector of control inputs for post-processing
% 
% There should be a script called create_rolling_spider.m which sets up the
% parameters for rolling spider. If not, it is easy to create one. All the
% variables here are defined somewhere in your code already, its just a
% matter of reorganizing them. This format makes it easy to change which
% controller you are implementing, as well as making it very easy to switch
% to aircraft later. It also saves you having to carry around lots of
% global variables or passing in tons of extra parameters to ode45 or
% having to hard-code in parameters in your integrator. This integrator
% will work for ANY quadcopter, and with some small modifications can be
% made to work with aircraft. This is desirable because ultimately the
% rigid body equations of motion are the same no matter what vehicle you
% simulate.

% expand states
u = x(4);
v = x(5);
w = x(6);
phi   = x(7);
theta = x(8);
psi   = x(9);
p = x(10);
q = x(11);
r = x(12);

% expand parameters
Ix = vehicle.params.I(1,1);
Iy = vehicle.params.I(2,2);
Iz = vehicle.params.I(3,3);
eta   = vehicle.params.eta;
zeta  = vehicle.params.zeta;
alpha = vehicle.params.alpha;
beta  = vehicle.params.beta;
m = vehicle.params.m;

% expand environment variables
g = vehicle.environment.g;
Wx = vehicle.environment.Wx;
Wy = vehicle.environment.Wy;
Wz = vehicle.environment.Wz;

% get the current controls
[Lc, Mc, Nc, Xc, Yc, Zc, f] = vehicle.controller(x);

% compute the resultant external force form aerodynamics
Xa = -eta*u^2*sign(u);
Ya = -eta*v^2*sign(v);
Za = -zeta*w^2*sign(w);

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
udot = (X - m*g*sin(theta))/m - q*wE + r*vE;
vdot = (Y + m*g*cos(theta)*sin(phi))/m - r*uE + p*wE;
wdot = (Z + m*g*cos(theta)*cos(phi))/m - p*vE + q*uE;

% compute derivatives of the euler angles
phidot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
thetadot = q*cos(phi) - r*sin(phi);
psidot = (q*sin(phi) + r*cos(phi))*sec(theta);

% compute aerodynamic moments
La = -alpha*p^2*sign(p);
Ma = -alpha*q^2*sign(q);
Na = -beta*r^2*sign(r);

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
  
% reassign control output
u = f;

end