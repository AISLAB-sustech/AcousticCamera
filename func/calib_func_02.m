function error = calib_func_02(g,numIterations)
% This file performs least square SLAM

%% parameters

% display starting time delay estimation error?
display_delay_error_on = 0;
% display norm(dx) for each iteration?
display_norm_dx_on = 0;

% the maximum number of iterations
% numIterations = 50;

% maximum allowed dx
EPSILON = g.eps;%1.5/1e-2;

% Error
err = 0;
%% start slSLAM
% 
% p2 = [g.xmm(:,1),g.xmm(:,10),g.xmm(:,18)];
% carry out the iterations
for i = 1:numIterations
    
    compute_global_error_02(g);

    disp(['Performing iteration ', num2str(i)]);
    if compute_global_error_02(g)<1e-5
       disp('error is small enough, iteration ends.');
      break
   end
  % solve the dx
  [dx,H] = linearize_and_solve_with_H_02(g);

  % TODO: apply the solution to the state vector g.x
  g.x = g.x + dx;
      
  % display estimation error of mic delay if asked
  if display_delay_error_on > 0    
      x_3_error = (g.x(9:5:g.M*5-1) - g.x_gt(9:5:g.M*5-1));
      disp('estimation error of starting time delay: ');
      x_3_error'
  end

  % compute current error
  err = compute_global_error_02(g);

  % TODO: implement termination criterion as suggested on the sheet
  if display_norm_dx_on>0
    disp(['norm(dx) = ' num2str(norm(dx))]);
  end
  

  if (norm(dx)<EPSILON)
      disp("norm(dx)<EPSILON, iteration ends.")
    break;
  end
  error(i) = error_analysis_02(g);

end
plot_02(g)
% plot the current state of the graph
% figure;
% plot_graph_with_cov(g, i, H);

end

