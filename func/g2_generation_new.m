% load the graph into the variable "g"
function g = g2_generation_new(num,R,t,init)

load("cameraParams/new_cameraParams_"+num+".mat");

cameraParams.NumPatterns;
K = 6*cameraParams.NumPatterns;
g.K = K;
g.eps = 1e-04;
g.M = 8;
g.x_gt = zeros(3*g.M,1);
g.xmm_gt = zeros(3*g.M,1);
g.x = zeros(length(g.x_gt),1);

xm_mic = [0,0,0;
          0.37,0,0;
          0,0.4,0;
          0,-0.006,0.395;
          0.365,0.385,0;
          0.37,-0.006,0.395;
          -0.002,0.395,0.395;
          0.39,0.395,0.395];
% xm_mic = [0,0,0;
%           0.5,0,0;
%           0,0.5,0;
%           0,0,0.5;
%           0.5,0.5,0;
%           0.5,0,0.5;
%           0,0.5,0.5;
%           0.5,0.5,0.5];

theta = 0;
Rm = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
g.Rm = Rm;
% Tm = [-0.17;-0.27;-0.21];
Tm = [-0.18;-0.28;-0.24];
% Tm = [0.25;0.25;0.25];
% Tm = t;
% g.x = init;
for i = 1:g.M        
    g.x_gt(3*(i-1)+1:3*(i-1)+3) = Rm*xm_mic(i,:)'+Tm;
    g.xmm_gt(3*(i-1)+1:3*(i-1)+3) = xm_mic(i,:)';
    % g.x(3*(i-1)+1:3*(i-1)+3) = 0.4*rand(3,1);
    g.x(3*(i-1)+1:3*(i-1)+3) = R*init(3*(i-1)+1:3*(i-1)+3)+t;
end

%% 扬声器真值
speaker = [195,-2,0; -40,-5,5; 195,140,5; -40,140,2; 200,60,5; -40,60,5];

j = 1;
for num = 1:cameraParams.NumPatterns
    ex = cameraParams.PatternExtrinsics(num);
    R = ex.R;
    t = ex.Translation;

    for i = 1:size(speaker,1)
        xs_cam = R * speaker(i,:)'/1000 + t'/1000;                           
        xs_cam_all((j-1)*3+1:(j-1)*3+3) =  xs_cam;
        j = j + 1;
    end
end
g.xs_gt = xs_cam_all';
g.xs = g.xs_gt;

g.edges = struct('measurement',{},'information',{},'fromIdx',{},'toIdx',{});

for eid = 1:K
    g.edges(eid).fromIdx = 1;
    g.edges(eid).toIdx = 3*(eid-1)+1;
end

sigma = 1e-4;

for eid = 1:K
    x = g.xs_gt(g.edges(eid).toIdx: g.edges(eid).toIdx+2);   % the robot pose
    l = g.x_gt(g.edges(eid).fromIdx: g.edges(eid).fromIdx+(3*g.M-1));     % the landmark
    T_k = zeros(g.M*(g.M-1)/2,1);
    k=1;
    for i = 1:g.M-1

        d_ik = sqrt((x(1)-l(3*(i-1)+1))^2 + (x(2)-l(3*(i-1)+2))^2 + (x(3)-l(3*(i-1)+3))^2);

        for j = i+1:g.M

            d_jk=sqrt((x(1)-l(3*(j-1)+1))^2 + (x(2)-l(3*(j-1)+2))^2 + (x(3)-l(3*(j-1)+3))^2);

            T_ijk = d_jk/340-d_ik/340;

            dis_mat(eid,k) = T_ijk;

            T_k(k) = T_ijk;
            k = k + 1;
        end
    end
    g.edges(eid).measurement = T_k;
    g.edges(eid).information = eye(g.M*(g.M-1)/2)*(1/sigma)^2;
end
save('dis_02_new.mat','dis_mat');
save('data_02.mat','g');
end