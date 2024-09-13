function g = data02_generation(g,SNR,sigma)
load("data_02.mat");
epsilon = 10^(-SNR/20);
if sigma == 0
    for eid = 1:g.K
        T_k = g.edges(eid).measurement;
        sigma = epsilon*rms(T_k);
        
        g.edges(eid).measurement = T_k+sigma*rand(g.M*(g.M-1)/2,1);
        g.edges(eid).information = eye(g.M*(g.M-1)/2)*(1/sigma)^2;
        g.sigma(eid,1) = sigma;
    end
end
if SNR == 0
     for eid = 1:g.K
        T_k = g.edges(eid).measurement;
        % sigma = epsilon*rms(T_k);
        
        g.edges(eid).measurement = T_k+sigma*rand(g.M*(g.M-1)/2,1);
        g.edges(eid).information = eye(g.M*(g.M-1)/2)*(1/sigma)^2;
    end
end
end