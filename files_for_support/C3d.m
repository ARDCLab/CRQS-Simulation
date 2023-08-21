function C_ba = C3(angle)
%C3  Determines the DCM for a 3-rotation of a desired angle.
% C_ba = C3d(angle) calculates the DCM required for a 3-rotation of a
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

C_ba = [cosd(angle) sind(angle) 0; -sind(angle) cosd(angle) 0; 0 0 1];

end

