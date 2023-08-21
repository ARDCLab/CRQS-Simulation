function u = UncrossOp(u_cross)
%UNCROSSOP  Transforms a cross operator matrix into its 3x1 matrix.
% u = UncrossOp(u_cross) calculates the 3x1 matrix from a cross operator.
%
% INPUT PARAMETERS:
% u_cross = cross operator 
% u_cross = [  0   -u(3)  u(2);...
%             u(3)   0   -u(1);
%            -u(2)  u(1)   0   ];
%
% OUTPUT PARAMETERS:
% u = any 3x1 matrix describing physical vector u in a frame

% William Elke
% Updated 26-Feb-2020
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u = [u_cross(3,2);
     u_cross(1,3);
     u_cross(2,1)];
 
end

