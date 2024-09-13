function MSE = error_analysis_01(g)
m=0;
for i = 4:3:3*(g.M-1)
    m_x = (g.x(i)-g.x_gt(i))^2;
    m_y = (g.x(i+1)-g.x_gt(i+1))^2;
    m_z = (g.x(i+2)-g.x_gt(i+2))^2;
    m = m + m_x+m_y+m_z;    
end
MSE = m/(g.M-1)
end