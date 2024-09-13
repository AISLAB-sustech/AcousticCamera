% plot a 3D SLAM graph
function plot_00(g)

if nargin<2
iteration = -1;
end

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



% g.x(1:3) = R*g.x(1:3);
% g.x_gt(1:3) = R*g.x_gt(1:3);
l = 0:3:3*g.M-1;

if (length(l) > 0)
  landmarkIdxX = l+1;
  landmarkIdxY = l+2;
  landmarkIdxZ = l+3;

  plot3(g.x(landmarkIdxZ), g.x(landmarkIdxX), g.x(landmarkIdxY), 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','g', 'LineWidth',2);
  plot3(g.x_gt(landmarkIdxZ), g.x_gt(landmarkIdxX), g.x_gt(landmarkIdxY), 'LineStyle','none','Marker','o', 'MarkerSize', 4,'MarkerEdgeColor','r', 'LineWidth',2);
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
  plot3(g.xs(pIdxZ), g.xs(pIdxX), g.xs(pIdxY), 'LineStyle','none','Marker','s', 'MarkerSize', 4,'MarkerEdgeColor','c', 'LineWidth',2);
  % plot3(g.xs_gt(pIdxZ), g.xs_gt(pIdxX), g.xs_gt(pIdxY),'Color','magenta', 'LineStyle','none','Marker','o', 'MarkerSize', 4,'MarkerEdgeColor','magenta', 'LineWidth',2);
end
theta = 0;
Rm = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];

rotationMatrix = Rm*[0,-1,0;
                  0,0,-1;
                  1,0,0];



% 单位矩阵表示无旋转
translationVector = [0, 0, 0]; % 原点
plotCamera('Size', 0.05, 'Orientation', rotationMatrix, 'Location', translationVector, 'Color', 'b', 'Opacity', 0.1,'AxesVisible', true);

legend({'Mic. pos. est.','Mic. pos. g. t.','Sound source est.'},'Location', 'Best');
% legend('show', 'Location', 'northwest');
grid on;
axis equal
view(-162,13);
hold off;

gcf;
grid on; axis equal;
xlabel('Z (m)');ylabel('X (m)');zlabel('Y (m)');

drawnow;

end
