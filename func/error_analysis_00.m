function MSE = error_analysis_00(g)
m=0;
for i = 1:3:3*(g.M-2)
    m_x = (g.xmm(i)-g.xmm_gt(i))^2;
    m_y = (g.xmm(i+1)-g.xmm_gt(i+1))^2;
    m_z = (g.xmm(i+2)-g.xmm_gt(i+2))^2;
    m = m + m_x+m_y+m_z;    
end
MSE = m/(g.M-1);
end