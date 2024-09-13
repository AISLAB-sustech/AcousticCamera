function axis_angle_vector = rotationMatrixToAxisAngleVector(R)
    % 计算旋转角度
    theta = acos((trace(R) - 1) / 2);
    
    % 计算旋转轴向矢量
    if sin(theta) ~= 0
        % 使用罗德里格公式
        axis = (1 / (2 * sin(theta))) * [R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
    else
        % 如果角度接近于0或pi，直接计算轴向矢量
        [V, D] = eig(R);
        axis = V(:, find(abs(diag(D) - 1) < 1e-10));
    end
    
    % 计算旋转轴角向量
    axis_angle_vector = theta * axis;
end