% load the graph into the variable "g"
function g = g3_generation_18(num)
% num=68;
load("./cameraParams/cameraParams_"+num+".mat");
% load("measurement_"+num+".mat");
% load("../cameraParams/cameraParams_03.mat");
cameraParams.NumPatterns;
K = 6*cameraParams.NumPatterns;
g.K=K;
g.eps = 1e-03;
g.M = 8;
g.x_gt = zeros(3*g.M,1);
g.xmm_gt = zeros(3*(g.M-1),1);
g.x = zeros(length(g.x_gt),1);

xm_mic = [0.06,-0.35,-0.34;
          0.06,0.04,-0.34;
        -0.195,-0.35,0.015;
        0.3,-0.35,-0.19;
        -0.195,0.04,0.015;   
        0.3,0.04,-0.19;
         0.15,-0.35,0.16;
         0.15,0.04,0.16];

for i = 1:g.M        
    g.x_gt(3*(i-1)+1:3*(i-1)+3) = xm_mic(i,:)';
    g.x(3*(i-1)+1:3*(i-1)+3) = xm_mic(i,:)';
end

%% 扬声器真值
speaker = [200,0,2; -40,0,2; 200,140,2; -40,140,2; 200,60,2; -40,60,2];

n = 1;

for num = 1:cameraParams.NumPatterns
    ex = cameraParams.PatternExtrinsics(num);
    R = ex.R;
    t = ex.Translation;

    for i = 1:size(speaker,1)
        xs_cam = R * speaker(i,:)'/1000 + t'/1000;                     
        xs_cam_all((n-1)*3+1:(n-1)*3+3) =  xs_cam;
        n = n + 1;
    end

end
g.xs_gt = xs_cam_all';
g.xs = g.xs_gt;

g.edges = struct('measurement',{},'information',{},'fromIdx',{},'toIdx',{});

for eid = 1:K
    g.edges(eid).fromIdx = 1;
    g.edges(eid).toIdx = 3*(eid-1)+1;
end

sigma = 1;
for eid = 1:K
    x = g.xs_gt(g.edges(eid).toIdx: g.edges(eid).toIdx+2);   % the robot pose
    l = g.x_gt(g.edges(eid).fromIdx: g.edges(eid).fromIdx+(3*g.M-1));     % the landmark
    T_k = zeros(g.M-1,1);
    for n = 1:g.M-1
        d_nk=sqrt((x(1)-l(3*(n-1)+1))^2 + (x(2)-l(3*(n-1)+2))^2 + (x(3)-l(3*(n-1)+3))^2);
        d_1k=sqrt((x(1)-l(22))^2 + (x(2)-l(23))^2 + (x(3)-l(24))^2);       
        T_nk = d_nk/340-d_1k/340;
        dis_mat(eid,n) = T_nk;
        T_k(n) = T_nk;
    end
    g.edges(eid).measurement = T_k;
    g.edges(eid).information = eye(g.M-1)*(1/sigma)^2;
end
save('dis_03_18.mat','dis_mat');
% save('data_03.mat','g');
end