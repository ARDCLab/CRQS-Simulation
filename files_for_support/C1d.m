function C_ba = C1d(angle)
%C1  Determines the DCM for a 1-rotation of a desired angle.
% C_ba = C1d(angle) calculates the DCM required for a 1-rotation of a
% desired angle.
%
% INPUT PARAMETERS:
% angle = [deg] desired angle of rotation
%
% OUTPUT PARAMETERS:
% C_ba = DCM describing the rotated frame Fb resolved in the original frame
% Fa

% William Elke
% Updated 28-May-2021
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C_ba = [1 0 0; 0 cosd(angle) sind(angle); 0 -sind(angle) cosd(angle)];

end

