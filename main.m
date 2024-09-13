%% refresh
clear;
close all;
clc;

% add path for including some tool functions
addpath('func');

%% params
disp('begin');
disp('------------------------------------------------------------------');

%% 真实实验
N = 69;
n = 1;

Error_0 = zeros(n,1);
Error_1 = zeros(n,1);
Error_2 = zeros(n,1);
Error_3 = zeros(n,1);
tic

b = 0.1;
err = 0;    
k = 0;
init = zeros(24,1);
num=69;

Tm = [-0.18;-0.28;-0.24];

for i = 1:n
    theta = 0;
    R = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
    t = Tm - b/2 + b * rand(3,1);

    for m = 1:8      
        init(3*(m-1)+1:3*m) = rand(3,1)*0.5;
    end

 
    %%%%%%%%%%%%
    load("./data/my_measurement_2/my_measurement_2_"+69+"_new.mat");       
    g2 = g2_generation_new(num,R,t,init);

    for eid = 1:g2.K     
        g2.edges(eid).measurement = measurement(eid,:);
    end

    error = calib_func_02(g2,50);
    if error(end) > 1
        i=i-1;
        % break;
    end
    Error_2(i) = error(end)^0.5;

end
disp('Finish');
toc
