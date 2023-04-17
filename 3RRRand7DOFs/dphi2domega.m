function R_dphi2domega = dphi2domega(phi)
%dphi2domega 将z-y-z角的微分转化为角速度（转动微分）
%   输入：z-y-z欧拉角
%   输出：转化矩阵
R_dphi2domega=[
    0   -sin(phi(1))    sin(phi(2))*cos(phi(1))
    0   cos(phi(1))     sin(phi(2))*sin(phi(1))
    1   0               cos(phi(2))
];
end

