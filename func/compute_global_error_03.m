% Computes the total error of the graph
function Fx = compute_global_error_03(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);
  % pose-landmark constraint
l = g.x(edge.fromIdx:edge.fromIdx+(3*g.M-1));  % the landmark
x = g.xs(edge.toIdx:edge.toIdx+2);      % the robot pose

% compute the error of the constraint and add it to Fx.
% Use edge.measurement and edge.information to access the
% measurement and the information matrix respectively.
e_il = zeros(g.M-1,1);
n = 1;
for i= 1:1
    for j = i+1:g.M    
         e_il(n) = sqrt((l(3*(j-1)+1)-x(1))^2 + (l(3*(j-1)+2)-x(2))^2 + (l(3*(j-1)+3)-x(3))^2)/340 - ...
            sqrt((l(3*(i-1)+1)-x(1))^2 + (l(3*(i-1)+1)-x(2))^2 + (l(3*(i-1)+1)-x(3))^2)/340 ...
             - edge.measurement(n);
         n=n+1;
    end
end
e_ls_il = e_il' * edge.information * e_il;
Fx = Fx + e_ls_il;

end
