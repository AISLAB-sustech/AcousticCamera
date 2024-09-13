function  g = calib_func_lm(g,numIterations)
% This file performs least square SLAM

%% parameters

% maximum allowed dx
% EPSILON = g.eps;
EPSILON = g.eps;
%% start SLAM
% gonna estimate clock drift?
est_drift_on = 1;
% gonna estimate starting time delay?
est_delay_on = 1;
% display starting time delay estimation error?
display_delay_error_on = 0;
% display norm(dx) for each iteration?
display_norm_dx_on = 0;
% carry out the iterations
y = [];
num = 1;
I = eye(length(g.xs));
i = 0;
% [H,b] = linearize_and_solve_with_H(g);
u = 1e+4;
v = 2;
err = [];
tic
if numIterations>0
    while num<numIterations
        compute_global_error_01(g);
        i = i+1;
        
        disp(['Performing iteration ', num2str(num)]);
      

      if compute_global_error_01(g)<1e-5
          num=num-1;
          disp('error is small enough, iteration ends.');
          break
      end

      % solve the dx
      [H,b] = linearize_and_solve_with_H_03(g,est_delay_on,est_drift_on);
      
      G = H + u*I;
      dx = G\(-b);

      err = [err,compute_global_error(g)];
    
      if norm(dx)>4
%           disp([num2str(norm(dx)),' norm(dx)>4'])
          u = u * 100;
          if length(err)>1
            err = err(1:end-1);
          end
          continue;
      end
             
      rho = [];
      if length(err)>1 
            rho = (err(length(err)-1)-err(length(err)))/(dx'*(u*dx-b));

          if rho>0.75
              % disp([num2str(rho),' rho>0.75'])
                u = u * max(1/3, 1-2*(rho-1)^3);
                v = 2;
          elseif rho<0.25 && rho>0
              % disp([num2str(rho),' 0<rho<0.25'])
                u = u * v;
                v = v * 2;
          elseif rho < 0 && rho > -2
              % disp([num2str(rho),' -2<rho<0'])
                u = u * 2;
                err = err(1:end-1); % do not iterate
                continue
          elseif rho<-2
              % disp([num2str(rho),' rho<-2,break'])
              break
          end
      end
      norm(dx);
      y = [y;norm(dx)];
      % TODO: apply the solution to the state vector g.x
      g.xs = g.xs + dx;
      
      num = num +1;

      if (norm(dx)<EPSILON)
        disp("norm(dx)<EPSILON, iteration ends.")
        break
      end
          
      if (isnan(norm(dx)))
          disp("norm(dx)=NaN, iteration ends.")
          break
      end

    end
    
end
toc
end

