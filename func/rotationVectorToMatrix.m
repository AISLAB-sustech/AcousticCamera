function rotation_matrix = rotationVectorToMatrix(rotation_vector)
    theta = norm(rotation_vector);
    k = rotation_vector / theta;
    K = [0, -k(3), k(2); k(3), 0, -k(1); -k(2), k(1), 0];
    rotation_matrix = eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2;
    if norm(rotation_vector) < 0.01
        rotation_matrix = eye(3);
    end

end