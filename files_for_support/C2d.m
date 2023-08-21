function C_ba = C2d(angle)
%C2  Determines the DCM for a 2-rotation of a desired angle.
% C_ba = C2d(angle) calculates the DCM required for a 2-rotation of a
% desired angle.
%
% INPUT PARAMETERS:
% angle = [deg] desired angle of rotation
%
% OUTPUT PARAMETERS:
% C_ba = DCM describing the rotated frame Fb resolved in the original frame
% Fa

% William Elke
% Updated 28-May-2020
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C_ba = [cosd(angle) 0 -sind(angle); 0 1 0; sind(angle) 0 cosd(angle)];

end

