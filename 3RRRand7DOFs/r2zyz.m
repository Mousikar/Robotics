function [alpha,beta,gamma] = r2zyz(R)
% 旋转矩阵转为zyz角
%   输入：旋转矩阵3*3
%   输出：zyz角
r31=R(3,1);
r32=R(3,2);
r33=R(3,3);
r23=R(2,3);
r13=R(1,3);
r11=R(1,1);
r12=R(1,2);
alpha=atan2(r23,r13);
beta=atan2(sqrt(r31^2+r32^2),r33);
gamma=atan2(r32,-r31);
if beta==0||beta==pi
    alpha=0;
    gamma=atan2(-r12,r11);
end
end

