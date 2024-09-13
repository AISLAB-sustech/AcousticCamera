% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function [dx,H] = linearize_and_solve_with_H_03(g)

% nnz = nnz_of_graph(g);
% 
% % allocate the sparse H and the vector b
% H = spalloc(length(g.x), length(g.x), nnz);
H = zeros(length(g.x));
b = zeros(length(g.x), 1);


% compute the addend term to H and b for each of our constraints
for eid = 1:length(g.edges)
  edge = g.edges(eid);
  % pose-landmark constraint
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.xs_gt(edge.toIdx:edge.toIdx+2);  % the robot pose
    x2 = g.x(edge.fromIdx:edge.fromIdx+(3*g.M-1));     % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A] = linearize_pose_landmark_constraint_03(x1, x2, edge.measurement,g);
    
    
    % compute and add the term to H and b
    b(edge.fromIdx:edge.fromIdx+(3*g.M-1)) = (b(edge.fromIdx:edge.fromIdx+(3*g.M-1))' + (e')*edge.information*A)';
    

    H(edge.fromIdx:edge.fromIdx+(3*g.M-1),edge.fromIdx:edge.fromIdx+(3*g.M-1)) = H(edge.fromIdx:edge.fromIdx+(3*g.M-1),edge.fromIdx:edge.fromIdx+(3*g.M-1)) + A'*edge.information*A;

end

% solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H

dx = H\(-b);
save('H.mat','H')
save('A.mat','A')
end
