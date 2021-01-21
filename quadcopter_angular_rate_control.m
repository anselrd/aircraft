function [L, M, N, X, Y, Z, u] = ...
    quadcopter_angular_rate_control(x,kp,kq,kr,vehicle)

R = vehicle.params.thruster_locs;
k = vehicle.params.k;
m = vehicle.params.m;
g = vehicle.environment.g;

p = x(10);
q = x(11);
r = x(12);

A = [ -R(:,2)'; -R(:,1)'; -k*R(:,4)'; -1*ones(1,4) ];
b = [ -kp*p; -kq*q; -kr*r; -m*g ];

u = A\b;
L = b(1);
M = b(2);
N = b(3);
X = 0;
Y = 0;
Z = b(4);

end