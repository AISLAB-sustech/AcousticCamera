function g = trans_calib_func(g,numIterations)

dt = 0.01;
dw = 0.001;
sign = -1;
Err = [];

for i = 1:numIterations 

    disp(['Performing iteration ', num2str(i)]);
    Err = compute_global_error_00(g);

    % wm
    for index = 1:3
        % disp(['minimised error in component ',num2str(index),' of wm']);
        for j = 1:10
            % disp(['j = ', num2str(j)]);
            w0 = g.wm;
            g.wm(index) = g.wm(index)+sign*dw;
            wm = g.wm;

            err = compute_global_error_00(g);
    
            Err = [Err err];

            if length(Err)>1
                if  Err(end) - Err(end-1) > 0
                    % disp('increase, reject the latest value');
                    g.wm = w0;
                    Err = Err(1:end-1);
                    if length(Err) < 4
                        % disp('increase on the first iteration of 3');
                        sign = -sign;
                    else
                        % disp('find');
                        wm = g.wm;
                        break;
                    end
                else
                    % disp('decrease, go on');
                end
            end
        end
        disp('search done');
        wm = g.wm
    end 

    % Tm
    for index = 1:3
        % disp(['minimised error in component ',num2str(index),' of Tm']);
        for j = 1:10
            % disp(['j = ', num2str(j)]);
            T0 = g.Tm;
            g.Tm(index) = g.Tm(index)+sign*dt;
            Tm = g.Tm;
 
            err = compute_global_error_00(g);
    
            Err = [Err err];

            if length(Err)>1
                if  Err(end) - Err(end-1) > 0
                    % disp('increase, reject the latest value');
                    g.Tm = T0;
                    Err = Err(1:end-1);
                    if length(Err) < 4
                        % disp('increase on the first iteration of 3,');
                        sign = -sign;
                    else
                        % disp('find');
                        Tm = g.Tm;
                        break;
                    end
                else
                     % disp('decrease, go on');
                end
            end
        end
        disp('search done');
        Tm = g.Tm
    end 
    dt = dt/10;
    dw = dw/10;
if dt < 1e-5
    break;
end
end


end