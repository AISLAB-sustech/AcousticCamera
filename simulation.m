%% refresh
clear;
close all;
clc;

% add path for including some tool functions
addpath('func');

%% params
disp('begin');
disp('------------------------------------------------------------------');

num = 100;
SNR = 0 ;
MSE = zeros(num,1);
rmse = zeros(length(SNR),1);
Error = 0;

xm_mic = [0,0,0;
          0.37,0,0;
          0,0.4,0;
          0,-0.006,0.395;
          0.365,0.385,0;
          0.37,-0.006,0.395;
          -0.002,0.395,0.395;
          0.39,0.395,0.395];

a = 0.5;
b = 0.1;
err = 0;    
k = 0;
% Tm = [-0.18;-0.28;-0.24];
Tm = [0.2;0.2;0.2];

init = zeros(24,1);
theta = 0;
R = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
t = Tm - b/2 + b * rand(3,1);

for m = 1:8
    % init(3*(m-1)+1:3*m) = rand(3,1)*a*2-a + xm_mic(m,:)';
    init(3*(m-1)+1:3*m) = rand(3,1)*a;
end
SIG = 0.0000666*[1;5;15;20];
%%
tic
load("./data/my_measurement_1/my_measurement_1_"+69+"_new.mat");       
g0 = g0_generation_new(69,R,t,init);

for ii = 1:length(SIG)
    % Error = 0;
    % snr = SNR(ii);
    sigma = SIG(ii);
    for i = 1:num
        g0 = data00_generation(g0,0,sigma);
        g0 = trans_calib_func(g0,10);

        % g0.Tm = g0.Tm_gt;
        % g0.wm = g0.wm_gt;

        error = calib_func_00(g0);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    rmse(ii) = mean(MSE)^0.5;
end
save('MSE_00','rmse');
plot(SIG,rmse,'--r*','MarkerSize', 4, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
hold on



%%
load("./data/my_measurement_1/my_measurement_1_"+69+"_new.mat");       
g1 = g1_generation_new(69,R,t,init);

for ii = 1:length(SIG)
    % Error = 0;
    % snr = SNR(ii);
    sigma = SIG(ii);
    for i = 1:num
        g1 = data01_generation(g1,0,sigma);
        % load('data_00.mat');
        error = calib_func_01(g1,20);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    
    rmse(ii) = mean(MSE)^0.5;
end
save('MSE_01','rmse');
plot(SIG,rmse,'--bo','MarkerSize', 4, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue', 'LineWidth', 1.5);
hold on

%%
load("./data/my_measurement_3/my_measurement_3_"+69+"_new.mat");       
g3 = g3_generation_new(69,R,t,init);

for ii = 1:length(SIG)
    Error = 0;
    % snr = SNR(ii);
    sigma = SIG(ii);
    for i = 1:num
        g3 = data02_generation(g3,0,sigma);
        % load('data_00.mat');
        error = calib_func_02(g3,50);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    
    rmse(ii) = mean(MSE)^0.5;
    
    % plot(1:num,MSE,'-o','MarkerSize', 4, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue', 'LineWidth', 1.5);
    % hold on
end
save('MSE_03','rmse');
plot(SIG,rmse,'-mo','MarkerSize', 4, 'MarkerEdgeColor', 'm', 'MarkerFaceColor', 'm', 'LineWidth', 1.5);
hold on

%%
load("./data/my_measurement_2/my_measurement_2_"+69+"_new.mat");       
g2 = g2_generation_new(69,R,t,init);

for ii = 1:length(SIG)
    Error = 0;
    % snr = SNR(ii);
    sigma = SIG(ii);
    for i = 1:num
        g2 = data02_generation(g2,0,sigma);
        % load('data_00.mat');
        error = calib_func_02(g2,50);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    
    rmse(ii) = mean(MSE)^0.5;
    
    % plot(1:num,MSE,'-o','MarkerSize', 4, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue', 'LineWidth', 1.5);
    % hold on
end
save('MSE_02','rmse');
plot(SIG,rmse,'-ko','MarkerSize', 4, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineWidth', 1.5);
xlabel('SNR');
ylabel('RMSE');
legend('SNSR','Ours1','Ours2','Ours2');
toc
%%
% % SNR = 0:2:40;
% load('MSE_00.mat');
% plot(SIG,rmse,'--r*','MarkerSize', 4, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
% hold on
% load('MSE_01.mat');
% plot(SIG,rmse,'--bo','MarkerSize', 4, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue', 'LineWidth', 1.5);
% hold on
% load('MSE_03.mat');
% plot(SIG,rmse,'-mo','MarkerSize', 4, 'MarkerEdgeColor', 'm', 'MarkerFaceColor', 'm', 'LineWidth', 1.5);
% load('MSE_02.mat');
% plot(SIG,rmse,'-ko','MarkerSize', 4, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineWidth', 1.5);
% xlabel('TDOA noise');
% ylabel('RMSE');
% legend('SNSR','Ours1','Ours2','Ours3');