function C_ba = C1(angle)
%C1  Determines the DCM for a 1-rotation of a desired angle.
% C_ba = C1(angle) calculates the DCM required for a 1-rotation of a
% desired angle.
%
% INPUT PARAMETERS:
% angle = [rad] desired angle of rotation
%
% OUTPUT PARAMETERS:
% C_ba = DCM describing the rotated frame Fb resolved in the original frame
% Fa

% William Elke
% Updated 26-Feb-2020
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C_ba = [1 0 0; 0 cos(angle) sin(angle); 0 -sin(angle) cos(angle)];

end

