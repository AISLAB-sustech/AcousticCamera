function g = data03_generation(g,SNR,sigma)
load("data_03.mat");


if sigma == 0
    epsilon = 10^(-SNR/20);
    for eid = 1:g.K
        T_k = g.edges(eid).measurement;
        sigma = epsilon*rms(T_k);
        
        g.edges(eid).measurement = T_k+sigma*rand(g.M-1,1);
        g.edges(eid).information = eye(g.M-1)*(1/sigma)^2;
        g.sigma(eid,1) = sigma;
    end
end

if SNR == 0
    for eid = 1:g.K
        T_k = g.edges(eid).measurement;
        % sigma = epsilon*rms(T_k);
        
        g.edges(eid).measurement = T_k+sigma*rand(g.M-1,1);
        g.edges(eid).information = eye(g.M-1)*(1/sigma)^2;
    end
end
end