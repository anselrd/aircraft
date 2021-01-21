function [ vector_body ] = TransformFromInertialToBody( vector_inertial, euler_angles )
% Inputs: vector_inertial - vector of length 3 with the components of a
%                           vector in the inertial frame
%         euler_angles    - the 3-2-1 euler angles describing the
%                           transformation from inertial coordinates to
%                           body coordinates in the order phi theta psi
% Outputs: vector_body - the input vector in body coordinates
% Methodology: this function works by taking input and outputting output

% find the rotation matrix
R = RotationMatrix321( euler_angles );

% use the matrix to transform to body coordinates
if isvector(vector_inertial)
    vector_body = R*vector_inertial(:);
elseif size(vector_inertial,1) == 3
    vector_body = R*vector_inertial;
else
    vector_body = R*vector_inertial';
end