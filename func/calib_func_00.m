function error = calib_func_00(g)
% This file performs a numeric search algorithm
%% parameters
% the maximum number of iterations
numIterations = 50;

% maximum allowed dx
EPSILON = g.eps;%1.5/1e-2;
dx = 0.01;
sign = 1;
Err = 0;
%% start 
for iter = 1:numIterations
    Err = compute_global_error_00(g);

    % disp(['Performing iteration ', num2str(i)]);

    if compute_global_error_00(g)<1e-5
       disp('error is small enough, iteration ends.');
      break
    end
    dx = dx/1.1;

    for i = 1:(g.M-1)
        % disp(['adjust microphone ',num2str(i)]);        
        for index = 1:3
            % disp(['minimised error in component ',num2str(index),' of microphone ',num2str(i)]);
            for j = 1:20
                % disp(['j = ', num2str(j)]);
                xmm_index = g.xmm(3*(i-1)+index);
                g.xmm(3*(i-1)+index) = g.xmm(3*(i-1)+index)+sign*dx;
                xmm = g.xmm(3*(i-1)+index);
                err = compute_global_error_00(g);
                Err = [Err err];
                if length(Err)>1
                    if  Err(end) - Err(end-1) > 0
                        % disp('increase, reject the latest value');
                        g.xmm(3*(i-1)+index) = xmm_index;
                        Err = Err(1:end-1);
                        if length(Err) < 10
                            % disp('increase on the first iteration of 3');
                            sign = -sign;
                        else
                            % disp('find');
                            
                            break;
                        end
                    else
                        % disp('decrease, go on');
                    end
                end

            end
            % disp('search done');
        end

    end

  if (dx<EPSILON)
      disp("dx<EPSILON, iteration ends.")
    break;
  end
error(i) = error_analysis_00(g);
end

% plot_00(g);
end

