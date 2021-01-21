function [ L, M, N, X, Y, Z, u] = ...
    quadcopter_angle_angular_rate_control_linear(x,kt,ktd,kp,kpd,kr,vehicle)

R = vehicle.params.thruster_locs;
k = vehicle.params.k;
m = vehicle.params.m;
g = vehicle.environment.g;

phi = x(7);
theta = x(8);
phidot = x(10);
thetadot = x(11);
r = x(12);

A = [ -R(:,2)'; -R(:,1)'; -k*R(:,4)'; -1*ones(1,4) ];
b = [ -(kp*phi+kpd*phidot); -(kt*theta+ktd*thetadot); -kr*r; -(m*g-sum(vehicle.trim.f)) ];

u = A\b;
L = b(1);
M = b(2);
N = b(3);
X = 0;
Y = 0;
Z = b(4);

end