% plot a 3D SLAM graph
function plot_02(g)


clf;
% plotCamera('Location',[0,0,0],'Orientation',[1,0,0;0,1,0;0,0,1],'Size',0.1,...
%         'AxesVisible', true, 'Opacity', 0.1,'Color', 'r', 'Label', '相机');
% hold on
% plotCamera('Location',[0.3,0,-0.35],'Orientation',[1,0,0;0,1,0;0,0,1],'Size',0.1,...
%         'AxesVisible', true, 'Opacity', 0.1,'Color', 'r', 'Label', '相机');
% hold on
% plotCamera('Location',[-0.30,0,-0.35],'Orientation',[1,0,0;0,1,0;0,0,1],'Size',0.1,...
%         'AxesVisible', true, 'Opacity', 0.1,'Color', 'r', 'Label', '相机');
hold on;
% plot3(nan, nan, nan, 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','g', 'LineWidth',2);
% plot3(nan, nan, nan, 'Color','g');
% plot3(nan, nan, nan, 'LineStyle','none','Marker','o', 'MarkerSize', 4,'MarkerEdgeColor','r', 'LineWidth',2);
% plot3(nan, nan, nan, 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','c');
% plot3(nan, nan, nan, 'LineStyle','none','Marker','x', 'MarkerSize', 4,'MarkerEdgeColor','b');

% legend('Mic. pos. est.','Sigma region of mic. pos. est.','Mic. pos. g. t.','Sound source est.', 'Sound source g. t.');
% newxs = zeros(length(g.xs),1);
% theta = pi/12;
% R = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
% 
% for i = 1:length(g.xs)/3
%     newxs(3*(i-1)+1:3*(i-1)+3) = R*g.xs(3*(i-1)+1:3*(i-1)+3);
% end

% 
xs = zeros(length(g.xs),1);
% mic_gt = length(g.x_gt);
theta = -0.14;
Rm = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
for i=1:length(g.xs)/3
    
    g.xs(3*(i-1)+1:3*(i-1)+3) = Rm*g.xs(3*(i-1)+1:3*(i-1)+3);
    % mic_gt(3*(i-1)+1:3*(i-1)+3) = g.x_gt(3*(i-1)+1:3*(i-1)+3);
end





l = 0:3:3*g.M-1;

if (length(l) > 0)
  landmarkIdxX = l+1;
  landmarkIdxY = l+2;
  landmarkIdxZ = l+3;

  plot3(g.x(landmarkIdxZ), g.x(landmarkIdxX), -g.x(landmarkIdxY), 'LineStyle','none','Marker','s', 'MarkerSize', 6,'MarkerEdgeColor','g', 'LineWidth',2);
  plot3(g.x_gt(landmarkIdxZ), g.x_gt(landmarkIdxX), -g.x_gt(landmarkIdxY), 'LineStyle','none','Marker','o', 'MarkerSize', 6,'MarkerEdgeColor','r', 'LineWidth',2);
end



p = 0:3:3*length(g.edges)-1;
if (length(p) > 0)
  pIdxX = p+1;
  pIdxY = p+2;
  pIdxZ = p+3;
  % source = [g.xs(pIdxZ); g.xs(pIdxX); g.xs(pIdxY)];
  
  % newsource = R*source;
  % plot3(newsource(1), newsource(2), newsource(3), 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','c', 'LineWidth',2); 
  % s = scatter3(g.xs_gt(pIdxZ), g.xs_gt(pIdxX), g.xs_gt(pIdxY),'magenta','filled');
  % s.SizeData = 10;
  % plot3(newxs(pIdxZ), newxs(pIdxX), newxs(pIdxY),'Color','magenta', 'LineStyle','none','Marker','o', 'MarkerSize', 4,'MarkerEdgeColor','magenta', 'LineWidth',2);
  % plot3(newxs(pIdxZ), newxs(pIdxX), newxs(pIdxY), 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','c', 'LineWidth',2);
  % 
  % for i= 1:3:length(pIdxX)
  %     theta = -0.1;
  %     R = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
  %     Rs=R*[0,1,0;
  %           0,0,1;
  %           1,0,0];
  %     xs(i:i+2) = Rs*g.xs(i:i+2);
  % end
  % 
  % plot3(xs(pIdxZ), xs(pIdxX), xs(pIdxY), 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','c', 'LineWidth',2);

  plot3(g.xs(pIdxZ), g.xs(pIdxX), -g.xs(pIdxY), 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','c', 'LineWidth',2);
  % plot3(g.xs(pIdxZ), g.xs(pIdxX), g.xs(pIdxY),'Color','magenta', 'LineStyle','none','Marker','o', 'MarkerSize', 4,'MarkerEdgeColor','magenta', 'LineWidth',2);
end


% rotationMatrix = Rm*[0,1,0;
%                   0,0,1;
%                   1,0,0];
rotationMatrix = [0,-1,0;
                  0,0,-1;
                  1,0,0];



% 单位矩阵表示无旋转
translationVector = [0, 0, 0]; % 原点
plotCamera('Size', 0.05, 'Orientation', rotationMatrix, 'Location', translationVector, 'Color', 'k', 'Opacity', 0.1);

% 定义坐标轴长度
axisLength = 0.1;

% 绘制相机原点处的坐标轴
quiver3(0,0,0, ...
        0, -0.2, 0, ...
        'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X轴 (红色)

quiver3(0,0,0, ...
        0,0,-0.2, ...
        'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y轴 (绿色)

quiver3(0,0,0, ...
        0.2,0,0, ...
        'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z轴 (蓝色)




legend({'Mic. pos. est.','Mic. pos. g. t.','Sound source est.'},'Location', 'Best');
% legend('show', 'Location', 'northwest');
grid on;
axis equal
view(18,13);
hold off;

gcf;
grid on; axis equal;
xlabel('Z (m)');ylabel('X (m)');zlabel('Y (m)');
% set(gca, 'XTickLabel', {'0', '','0.1','','0.2','','0.3','','0.4','','0.5'})
set(gca, 'YTickLabel', {'0.2', '0','-0.2'})
set(gca, 'ZTickLabel', {'0.1', '0','-0.1','-0.2'})
drawnow;

end
