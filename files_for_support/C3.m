function C_ba = C3(angle)
%C3  Determines the DCM for a 3-rotation of a desired angle.
% C_ba = C3(angle) calculates the DCM required for a 3-rotation of a
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

C_ba = [cos(angle) sin(angle) 0; -sin(angle) cos(angle) 0; 0 0 1];

end

