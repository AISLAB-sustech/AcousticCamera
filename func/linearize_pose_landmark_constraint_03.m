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
function [e, J] = linearize_pose_landmark_constraint_03(x, l, z, g)

  % compute the error and the Jacobians of the error
   
  % error
  e = zeros(length(z),1);
  n = 1;
  for i = 1:1
      for j= i+1:g.M        
            e(n) = sqrt((l(3*(j-1)+1)-x(1))^2 + (l(3*(j-1)+2)-x(2))^2 + (l(3*(j-1)+3)-x(3))^2)/340 - ...
          sqrt((l(3*(i-1)+1)-x(1))^2 + (l(3*(i-1)+2)-x(2))^2 + (l(3*(i-1)+3)-x(3))^2)/340 - z(n);
            n = n+1;
      end
  end

  % computation of A, de/dx1, x1 here is landmark
  A = [];
  J = [];
  % A = [A zeros(g.M*(g.M-1)/2,3)];
  for i = 1:1
      A = zeros(g.M-i,3);

      for j = 1:(g.M-i)
          A(j,:) = -[(1/340)*((1/2)*(1/sqrt((l(3*(i-1)+1)-x(1))^2 + (l(3*(i-1)+2)-x(2))^2 + (l(3*(i-1)+3)-x(3))^2)) *(2*(l(3*(i-1)+1)-x(1)))),...
                  (1/340)*((1/2)*(1/sqrt((l(3*(i-1)+1)-x(1))^2 + (l(3*(i-1)+2)-x(2))^2 + (l(3*(i-1)+3)-x(3))^2)) *(2*(l(3*(i-1)+2)-x(2)))),...
                  (1/340)*((1/2)*(1/sqrt((l(3*(i-1)+1)-x(1))^2 + (l(3*(i-1)+2)-x(2))^2 + (l(3*(i-1)+3)-x(3))^2)) *(2*(l(3*(i-1)+3)-x(3))))];  
      end
    n = 1;
      for j = i+1:g.M
          
        A_struct(n).matrix = [zeros(n-1,3);
                              (1/340)*((1/2)*(1/sqrt((l(3*(j-1)+1)-x(1))^2 + (l(3*(j-1)+2)-x(2))^2 + (l(3*(j-1)+3)-x(3))^2)) *(2*(l(3*(j-1)+1)-x(1)))),...
                              (1/340)*((1/2)*(1/sqrt((l(3*(j-1)+1)-x(1))^2 + (l(3*(j-1)+2)-x(2))^2 + (l(3*(j-1)+3)-x(3))^2)) *(2*(l(3*(j-1)+2)-x(2)))),...
                              (1/340)*((1/2)*(1/sqrt((l(3*(j-1)+1)-x(1))^2 + (l(3*(j-1)+2)-x(2))^2 + (l(3*(j-1)+3)-x(3))^2)) *(2*(l(3*(j-1)+3)-x(3))));...                          ;
                              zeros(g.M-i-n,3)];    
        A = [A A_struct(n).matrix];
        n = n+1;
      end

      O = zeros(g.M-i,3*(i-1));

      J = [J;O A];

  end

  
  
end
