function [r, p, y] = DCM2Euler321(Cba)
%DCM2EULER321  DCM to Euler Angle conversion.
% [r, p, y] = DCM2Euler321(Cba) solves for the Euler Angles based on the
% DCM using expression on p. 23 of de Ruiter (2013).
%
% INPUT PARAMETERS:
% Cba = 3x3 DCM
%
% OUTPUT PARAMETERS:
% r = roll angle [rad]
% p = pitch angle [rad]
% y = yaw angle [rad]
%
% William Elke
% Updated 26-Feb-2020
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = atan2(Cba(2,3),Cba(3,3));
p = -asin(Cba(1,3));
y = atan2(Cba(1,2),Cba(1,1));

end