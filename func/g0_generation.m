% load the graph into the variable "g"
function g = g0_generation(num)
% num=68;
load("./cameraParams/cameraParams_"+num+".mat");
% load("measurement_"+num+".mat");
% load("../cameraParams/cameraParams_03.mat");
cameraParams.NumPatterns;
K = 6*cameraParams.NumPatterns;
g.K = K;
g.eps = 1e-03;
g.M = 9;
% g.x_gt = zeros(3*g.M,1);
g.xmm_gt = zeros(3*(g.M-1),1);
g.xmm = zeros(3*(g.M-1),1);
% g.x = zeros(length(g.x_gt),1);

% xm_mic = [];

xm_mic = [0.36,0.39,0;
          -0.1,0.39,0;
        0.35,0,0;
        0.35,0.38,0.37;
        0,0,0;
        -0.01,0.38,0.37;   
        0.35,0,0.37;
         0,0,0.38];

theta = -pi/30;
% theta = 0;
Rm = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
g.wm_gt = rotationMatrixToAxisAngleVector(Rm);
g.wm = [-0.1;0;0];
% g.wm = g.wm_gt;
% Rm = [1,0,0;0,1,0;0,0,1];
Tm = [-0.25;-0.32;-0.08];
g.Tm_gt = Tm;
% g.Tm = -rand(3,1);
g.Tm = Tm;
g.Tm = Tm -0.1 + (0.1 + 0.1) * rand(3, 1);

for i = 2:g.M        
    % g.x_gt(3*(i-1)+1:3*(i-1)+3) = Rm*xm_mic(i-1,:)'+Tm;
    g.xmm_gt(3*(i-2)+1:3*(i-2)+3) = xm_mic(i-1,:)';
    % g.xmm(3*(i-2)+1:3*(i-2)+3) = xm_mic(i-1,:)';
    g.xmm(3*(i-2)+1:3*(i-2)+3) = xm_mic(i-1,:)'-0.2 + (0.2 + 0.2) * rand(3, 1);
    % g.xmm(3*(i-2)+1:3*(i-2)+3) = rand(3,1);
end

%% 扬声器真值
speaker = [200,0,2; -40,0,2; 200,140,2; -40,140,2; 200,60,2; -40,60,2];
ref_mic = [-90,-45,-10];

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
     if num == 1
        ref_mic_cam = R * ref_mic'/1000 + t'/1000;
    end
end
g.xs_gt = xs_cam_all';
g.xs = g.xs_gt;
% g.x_gt(1:3) = ref_mic_cam;
% g.x(1:3) = ref_mic_cam;
g.ref = ref_mic_cam;

g.edges = struct('measurement',{},'information',{},'fromIdx',{},'toIdx',{});

for eid = 1:K
    g.edges(eid).fromIdx = 1;
    g.edges(eid).toIdx = 3*(eid-1)+1;
end

l(1:3) = ref_mic_cam;
for n = 1:(g.M-1)
    l(3*n+1:3*n+3) = rotationVectorToMatrix(g.wm)*g.xmm_gt(3*(n-1)+1:3*(n-1)+3)+g.Tm_gt;
end

sigma = 1e-5;

for eid = 1:K
    x = g.xs_gt(g.edges(eid).toIdx: g.edges(eid).toIdx+2);   % the robot pose
    % l = g.x_gt(g.edges(eid).fromIdx: g.edges(eid).fromIdx+(3*g.M-1));     % the landmark
    

    T_k = zeros(g.M-1,1);

    for n = 1:(g.M-1)
        
        d_nk=sqrt((x(1)-l(3*n+1))^2 + (x(2)-l(3*n+2))^2 + (x(3)-l(3*n+3))^2);
        d_1k=sqrt((x(1)-l(1))^2 + (x(2)-l(2))^2 + (x(3)-l(3))^2);       
        T_nk = d_nk/340-d_1k/340;
        dis_mat(eid,n) = T_nk;
        T_k(n) = T_nk;
    end
    g.edges(eid).measurement = T_k;

    g.edges(eid).information = eye(g.M-1)*(1/sigma)^2;
end
save('dis_00.mat','dis_mat');
save('data_00.mat','g');
end