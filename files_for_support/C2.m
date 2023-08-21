function C_ba = C2(angle)
%C2  Determines the DCM for a 2-rotation of a desired angle.
% C_ba = C2(angle) calculates the DCM required for a 2-rotation of a
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

C_ba = [cos(angle) 0 -sin(angle); 0 1 0; sin(angle) 0 cos(angle)];

end

