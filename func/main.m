%% refresh
clear;
close all;
clc;

% add path for including some tool functions
addpath('func');

%% params
disp('begin');
disp('------------------------------------------------------------------');

num = 10;
SNR = 1:1:40 ;
MSE = zeros(num,1);
mse = zeros(length(SNR),1);
Error = 0;
tic
%%
g0 = g0_generation(68);
for ii = 1:length(SNR)
    Error = 0;
    snr = SNR(ii);
    for i = 1:num
        g0 = data00_generation(g0,snr,0);
        g0 = trans_calib_func(g0,20);

        % g0.Tm = g0.Tm_gt;
        % g0.wm = g0.wm_gt;

        error = calib_func_00(g0);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    mse(ii) = mean(MSE);
end
save('MSE_00','MSE');
plot(SNR,mse,'-ks','MarkerSize', 4, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineWidth', 1.5);
hold on


%%
g1 = g1_generation(68);
for ii = 1:length(SNR)
    Error = 0;
    snr = SNR(ii);

    for i = 1:num
        g1 = data01_generation(g1,snr,0);
        % load('data_00.mat');
        error = calib_func_01(g1,20);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    
    mse(ii) = mean(MSE);
end
save('MSE_01','MSE');
plot(SNR,mse,'-bs','MarkerSize', 4, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue', 'LineWidth', 1.5);
hold on
%% 
g2 = g2_generation(68);
for ii = 1:length(SNR)
    Error = 0;
    snr = SNR(ii);

    for i = 1:num
        g2 = data02_generation(g2,snr,0);
        % load('data_00.mat');
        error = calib_func_02(g2,20);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    
    mse(ii) = mean(MSE);
    
    % plot(1:num,MSE,'-o','MarkerSize', 4, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue', 'LineWidth', 1.5);
    % hold on
end
save('MSE_02','MSE');
plot(SNR,mse,'-mo','MarkerSize', 4, 'MarkerEdgeColor', 'm', 'MarkerFaceColor', 'm', 'LineWidth', 1.5);
hold on

%%
g3 = g3_generation(68);
for ii = 1:length(SNR)
    Error = 0;
    snr = SNR(ii);

    for i = 1:num
        g = data03_generation(g3,snr,0);
        % load('data_00.mat');
        error = calib_func_03(g,20);
        % Error = Error + error(end);
        MSE(i) = error(end);
    end
    
    mse(ii) = mean(MSE);

end
save('MSE_03','MSE');
plot(SNR,mse,'-r>','MarkerSize', 4, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'LineWidth', 1.5);

legend('Dobler','ref','no-ref','no-ref-only-first-mic');
xlabel('SNR');
ylabel('MSE');

toc

% %% 真实实验
% load("./data/measurement01_68.mat");
% 
% g1 = g1_generation(68);
% for eid = 1:g1.K     
%     g1.edges(eid).measurement = measurement(eid,:);
% end
% error = calib_func_01(g1,50);
% 
% %%
% load("./data/measurement02_68.mat");
% g2 = g2_generation(68);
% for eid = 1:g2.K     
%     g2.edges(eid).measurement = measurement(eid,:);
%     % g2.edges(eid).information = eye(g.M*(g.M-1)/2);
% end
% error = calib_func_02(g2,50);
% 
% %%
% load("./data/measurement03_68.mat");
% g3 = g3_generation;
% for eid = 1:g3.K     
%     g3.edges(eid).measurement = measurement(eid,:);
%     % g2.edges(eid).information = eye(g.M*(g.M-1)/2);
% end
% error = calib_func_03(g3,50);