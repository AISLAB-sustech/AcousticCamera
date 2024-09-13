% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A] = linearize_pose_landmark_constraint_01(x, l, z,g)

  % compute the error and the Jacobians of the error
   
  % error
  e = zeros(g.M-1,1);
  for n = 1:(g.M-1)
    % e(n) = sqrt((l(5*n+1)-x(1))^2 + (l(5*n+2)-x(2))^2 + (l(5*n+3)-x(3))^2)/340 - ...
    %   sqrt((l(1)-x(1))^2 + (l(2)-x(2))^2 + (l(3)-x(3))^2)/340 + l(5*n+4) + ...
    %   ((toIdx+2)/3)*l(5*n+5) - z(n);
        e(n) = sqrt((l(3*n+1)-x(1))^2 + (l(3*n+2)-x(2))^2 + (l(3*n+3)-x(3))^2)/340 - ...
      sqrt((l(1)-x(1))^2 + (l(2)-x(2))^2 + (l(3)-x(3))^2)/340  - z(n);
  end
 
  % computation of A, de/dx1, x1 here is landmark
  A = [];
  A = [A zeros(g.M-1,3)];
  for n = 1:(g.M-1)
    A_struct(n).matrix = [zeros(n-1,3);
                          (1/340)*((1/2)*(1/sqrt((l(3*n+1)-x(1))^2 + (l(3*n+2)-x(2))^2 + (l(3*n+3)-x(3))^2)) *(2*(l(3*n+1)-x(1)))),...
                          (1/340)*((1/2)*(1/sqrt((l(3*n+1)-x(1))^2 + (l(3*n+2)-x(2))^2 + (l(3*n+3)-x(3))^2)) *(2*(l(3*n+2)-x(2)))),...
                          (1/340)*((1/2)*(1/sqrt((l(3*n+1)-x(1))^2 + (l(3*n+2)-x(2))^2 + (l(3*n+3)-x(3))^2)) *(2*(l(3*n+3)-x(3))));
                          
                          zeros(g.M-1-n,3)];
    A = [A A_struct(n).matrix];
  end
  
end
