% load the graph into the variable "g"
function g = g5_generation(num)
% num=68;
% load('cameraParams\cameraParams_68.mat');
load("cameraParams/cameraParams_"+num+".mat");
% load("measurement_"+num+".mat");
% load("../cameraParams/cameraParams_03.mat");
cameraParams.NumPatterns;
K = 6*cameraParams.NumPatterns;
g.K = K;
g.eps = 1e-04;
g.M = 8;
g.x_gt = zeros(3*g.M,1);
g.xmm_gt = zeros(3*g.M,1);
g.x = zeros(length(g.x_gt),1);
% 
% xm_mic = [0,0,0;
%           0.365,0,0;
%           0,0.4,0;
%           -0.01,-0.01,0.39;
%           0.36,0.39,0;
%           0.39,0.01,0.39;
%           -0.02,0.39,0.39;
%           0.39,0.39,0.38];

xm_mic = [0.36,0.39,0;
          -0.1,0.39,0;
        0.35,0,0;

        0,0,0;
        0.35,0.38,0.37;

        -0.01,0.38,0.37;   
        0.35,0,0.37;
         0,0,0.38];


Rm = rotationVectorToMatrix([0,0,0]);
% Rm = [-1,0,0;0,-1,0.5;0,0,0.9];
% Tm = [0.11;0.09;-0.16];

theta = -pi/30;
Rm = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
g.Rm = Rm;
Tm = [-0.25;-0.32;-0.08];
% Tm = [-0.25;-0.32;-0.08];
Tm = [-0.2;-0.25;-0.11];
for i = 1:g.M        
    g.x_gt(3*(i-1)+1:3*(i-1)+3) = Rm*xm_mic(i,:)'+Tm;
    % g.xmm_gt(3*(i-1)+1:3*(i-1)+3) = xm_mic(i,:)';
    % g.x(3*(i-1)+1:3*(i-1)+3) = rand(3,1);
    g.x(3*(i-1)+1:3*(i-1)+3) = Rm*xm_mic(i,:)';
end

%% 扬声器真值
speaker = [200,0,2; -40,0,2; 200,140,2; -40,140,2; 200,60,2; -40,60,2];

j = 1;
for num = 1:cameraParams.NumPatterns
    ex = cameraParams.PatternExtrinsics(num);
    R = ex.R;
    t = ex.Translation;

    for i = 1:size(speaker,1)
        xs_cam = R * speaker(i,:)'/1000 + t'/1000;            
        % s = scatter3(xs_cam(1),xs_cam(2),xs_cam(3),'magenta','filled');
        % s.SizeData = 10;
        % hold on           
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
    % g.edges(eid).information =eye(g.M-1)*1e+5;
end

sigma = 1e-4;
% A = [1 2 3 5];
% B = [4 6 7 8];
for eid = 1:K
    x = g.xs_gt(g.edges(eid).toIdx: g.edges(eid).toIdx+2);   % the robot pose
    l = g.x_gt(g.edges(eid).fromIdx: g.edges(eid).fromIdx+(3*g.M-1));     % the landmark
    T_k = zeros(16,1);
    k=1;
    for i = 1:4

        d_ik = sqrt((x(1)-l(3*(i-1)+1))^2 + (x(2)-l(3*(i-1)+2))^2 + (x(3)-l(3*(i-1)+3))^2);

        for j = 5:8

            d_jk=sqrt((x(1)-l(3*(j-1)+1))^2 + (x(2)-l(3*(j-1)+2))^2 + (x(3)-l(3*(j-1)+3))^2);

            T_ijk = d_jk/340-d_ik/340;

            dis_mat(eid,k) = T_ijk;
            % if ismember(i, A) && ismember(j, B)
            % 
             T_k(k) = T_ijk;
             k = k + 1;
            % end
            % 
        end

    end
    g.edges(eid).measurement = T_k;
    g.edges(eid).information = eye(16)*(1/sigma)^2;
end
save('dis_05.mat','dis_mat');
save('data_05.mat','g');
end