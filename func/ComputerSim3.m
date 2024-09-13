function [mT12i,mT21i] = ComputerSim3(P1,P2)
mbFixScale = false;
Pr1 = zeros(size(P1));
Pr2 = zeros(size(P2));

% O1 = zeros(3,1);
% O2 = zeros(3,1);

c1 = sum(P1, 2);
O1 = c1 / size(P1, 2);

c2 = sum(P2, 2);
O2 = c2 / size(P2, 2);

for i = 1:size(P1, 2)
    Pr1(:, i) = P1(:, i) - O1; % 减去质心
    Pr2(:, i) = P2(:, i) - O2;
end

M = Pr2*Pr1';

N11 = M(1,1) + M(2,2) + M(3,3);   % Sxx+Syy+Szz
N12 = M(2,3) - M(3,2);            % Syz-Szy
N13 = M(3,1) - M(1,3);            % Szx-Sxz
N14 = M(1,2) - M(2,1);            % ...
N22 = M(1,1) - M(2,2) - M(3,3);
N23 = M(1,2) + M(2,1);
N24 = M(3,1) + M(1,3);
N33 = -M(1,1) + M(2,2) - M(3,3);
N34 = M(2,3) + M(3,2);
N44 = -M(1,1) - M(2,2) + M(3,3);

N = [N11, N12, N13, N14;
     N12, N22, N23, N24;
     N13, N23, N33, N34;
     N14, N24, N34, N44];

[V, D] = eig(N); %返回特征值的对角矩阵 D 和矩阵 V
[eval,index] = sort(diag(D),'descend'); %特征值从大到小排列，所以evec[0] 是最大值
evec = zeros(4);
for i = 1:4
    n = index(i);
    evec(:,i) = V(:,n);
end

% N 矩阵最大特征值（第一个特征值）对应特征向量就是要求的四元数（q0 q1 q2 q3），其中q0 是实部
% 将(q1 q2 q3)放入vec（四元数的虚部）
vec = evec(2:4, 1)';

% Rotation angle. sin is the norm of the imaginary part, cos is the real part
% 四元数虚部模长 norm(vec)=sin(theta/2), 四元数实部 evec(1,1)=q0=cos(theta/2)
% 这一步的ang实际是theta/2，theta 是旋转向量中旋转角度
% ? 这里也可以用 acos(q0)=angle/2 得到旋转角吧
ang = atan2(norm(vec), evec(1,1));

% vec/norm(vec)归一化得到归一化后的旋转向量,然后乘上角度得到包含了旋转轴和旋转角信息的旋转向量vec
if norm(vec) > 0.00001    
    vec = 2 * ang * vec / norm(vec); % Angle-axis x. quaternion angle is the half
else
    vec = [0 0 0];
end

% 旋转向量（轴角）转换为旋转矩阵
mR12i = rotationVectorToMatrix(vec);

% Step 5: Rotate set 2
% 利用刚计算出来的旋转将三维点旋转到同一个坐标系，P3对应论文里的 r_l,i', Pr1 对应论文里的r_r,i'
P3 = mR12i * Pr2;

% Step 6: 计算尺度因子 Scale
if ~mbFixScale
    % 论文中有2个求尺度方法。一个是p632右中的位置，考虑了尺度的对称性
    % 代码里实际使用的是另一种方法，这个公式对应着论文中p632左中位置的那个
    % Pr1 对应论文里的r_r,i',P3对应论文里的 r_l,i',(经过坐标系转换的Pr2), n=3, 剩下的就和论文中都一样了
    nom = dot(Pr1, P3);
    nom = sum(nom);
    % 准备计算分母
    aux_P3 = P3.^2;
    den = sum(aux_P3(:));

    ms12i = nom / den;
else
    ms12i = 1;
end

% Step 7: 计算平移Translation
mt12i = O1 - ms12i*mR12i*O2;

% Step 8: 计算双向变换矩阵

% Step 8.1: 用尺度，旋转，平移构建变换矩阵 T12
mT12i = eye(4); % 初始化为单位矩阵

sR = ms12i * mR12i;

% 构建 T12 矩阵
mT12i(1:3, 1:3) = sR;
mT12i(1:3, 4) = mt12i;

% Step 8.2: 构建 T21 矩阵
mT21i = eye(4); % 初始化为单位矩阵

sRinv = (1.0 / ms12i) * mR12i'; % 逆旋转矩阵
mT21i(1:3, 1:3) = sRinv;
tinv = -sRinv * mt12i;
mT21i(1:3, 4) = tinv;

end