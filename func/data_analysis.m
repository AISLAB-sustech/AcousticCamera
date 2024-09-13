function MSE = data_analysis(g)

% xmc = g.x;
% 
% for i = 1:g.M
%     scatter3(xmc((i-1)*5+3),xmc((i-1)*5+1),xmc((i-1)*5+2),'black','filled');
%     hold on
% end
% % 定义相机位姿 (例如：旋转矩阵和平移向量)
% rotationMatrix = [0,1,0;
%                   0,0,1;
%                   1,0,0];
% % 单位矩阵表示无旋转
% translationVector = [0, 0, 0;
%                     -0.3,-0.2,0;
%                     -0.3,0.2,0]; % 原点
% 
% for i = 1:3
%     plotCamera('Size', 0.1, 'Orientation', rotationMatrix, 'Location', translationVector(i,:), 'Color', 'b', 'Opacity', 0.1);
% end
% xlabel('Z_c');
% ylabel('X_c');
% zlabel('Y_c');
% axis equal

s = 0;
for i = 6:5:5*g.M
    s_x = (g.x(i)-g.x_gt(i))^2;
    s_y = (g.x(i+1)-g.x_gt(i+1))^2;
    s_z = (g.x(i+2)-g.x_gt(i+2))^2;
    s = s + s_x+s_y+s_z;    
end
m = 0;
for i = 1:3:3*(g.M-1)
    m_x = (g.xmm(i)-g.xmm_gt(i))^2;
    m_y = (g.xmm(i+1)-g.xmm_gt(i+1))^2;
    m_z = (g.xmm(i+2)-g.xmm_gt(i+2))^2;
    m = m + m_x+m_y+m_z;    
end
MSE = m/(g.M-1);
end