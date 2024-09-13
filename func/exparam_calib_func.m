function g = exparam_calib_func(g)
XOZ = [5 3 8];
%求麦克风中心的坐标
xmc = zeros(3,g.M-1);
for i = 1:g.M-1
    xmc(:,i) = g.x(5*i+1:5*i+3);
end
 

O = xmc(:,XOZ(1));
P = xmc(:,XOZ(2));  
Q = xmc(:,XOZ(3));   

Om = [0;0;0];
Pm = [norm(P-O);0;0];
Qm = [0;norm(Q-O);0];
Qm = [0;0;norm(Q-O)];


p1 = [O,P,Q]; 
p2 = [Om,Pm,Qm];

[T,inv_T] = ComputerSim3(p1,p2)
g.xmm = zeros(length(g.xmm_gt),1);

for n = 2:g.M
   g.xmm(3*(n-2)+1:3*(n-2)+3) = [eye(3) zeros(3,1)]*inv_T*[g.x(5*(n-1)+1:5*(n-1)+3);1];
end
m=0;
for i = 1:3:3*(g.M-1)
    m_x = (g.xmm(i)-g.xmm_gt(i))^2;
    m_y = (g.xmm(i+1)-g.xmm_gt(i+1))^2;
    m_z = (g.xmm(i+2)-g.xmm_gt(i+2))^2;
    m = m + m_x+m_y+m_z;    
end
MSE = m/(g.M-1)
% MSE = m

end