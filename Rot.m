function [ R ] = Rot(n,theta)
%creates a rotation matrix about a specified axis n through angle theta
% Inputs: n - axis about which to rotate (1-3)
%         theta - angle [rad]
% Outputs: R - 3-by-3 rotation matrix

switch n
    case 1
        R = [1     0           0     ;...
             0 cos(theta)  sin(theta);...
             0 -sin(theta) cos(theta)];
    case 2
        R = [cos(theta) 0 -sin(theta);...
                 0      1      0     ;...
             sin(theta) 0 cos(theta)];
    case 3
        R = [cos(theta)  sin(theta) 0;
             -sin(theta) cos(theta) 0;
                  0          0      1];
    otherwise
        error('unexcpected dimension')
end

end