function vector_inertial = TransformFromBodyToInertial(vector_body, euler_angles)
% Inputs: vector_body  - vector of length 3 with the components of a vector
%                        in the body frame
%         euler_angles - the 3-2-1 euler angles describing the
%                        transformation from inertial coordinates to body 
%                        coordinates in the order phi theta psi
% Outputs: vector_inertial - the input vector in inertial coordinates
% Methodology: this function works by taking input and outputting output

% find the inverse rotation matrix
R = RotationMatrix321( euler_angles )';

% use matrix to transform from body to inertial coordinates
if isvector(vector_body)
    vector_inertial = R*vector_body(:);
elseif size(vector_body,1) == 3
    vector_inertial = R*vector_body;
else
    vector_inertial = R*vector_body';
end

end