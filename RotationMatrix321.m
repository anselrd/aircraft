function [ R ] = RotationMatrix321( euler_angles )
%Function that takes euler angles and creates a rotation matrix
% Inputs: euler_angles - the 3-2-1 euler angles describing the
%                        transformation in the order phi theta psi
% Outputs: R - the corresponding transformation matrix
% Methodology: creates single axis rotation matrices for each euler angle
%              then combines them

% take individual euler angles from input
phi = euler_angles(1); %roll
theta = euler_angles(2); %pitch
psi = euler_angles(3); %yaw

% create the three single-axis rotation matrices for each angle
R1 = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
R2 = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R3 = [cos(psi) sin(psi) 0 ; -sin(psi) cos(psi) 0; 0 0 1];

% multiply together for the full matrix
R = R1*R2*R3;


end

