function u_cross = CrossOp(u)
%CROSSOP  Transforms a 3x1 matrix into its cross operator
% u_cross = CrossOp(u) calculates the cross operator used to calculate the
% cross product between two physical vectors(u X v = u_cross*v).
%
% INPUT PARAMETERS:
% u = any 3x1 matrix describing physical vector u in a frame
%
% OUTPUT PARAMETERS:
% u_cross = cross operator

% William Elke
% Updated 26-Feb-2020
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u_cross = [  0   -u(3)  u(2);...
            u(3)   0   -u(1);
           -u(2)  u(1)   0   ];
       
end

