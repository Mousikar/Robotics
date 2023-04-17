function [medjoiang,u] = solveMedJoiAng(p,phi)
% 反解最中间的旋转关节变量
% 输入：机械臂末端位姿
% 位置：p=[px,py,pz].';
% 位姿：phi=[alpha beta gamma].';
% 输出：中间关节变量
% medjoiang(rad)
%% 参数：
[~,~,~,~,~,a_3,a_4,d_7] = paraconfig();
% a_3=255;% a_4=260;% d_1=50;
%% 求距离末端最近的三角形顶点
% 将末端位姿转化为旋转矩阵
Q=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
% 将旋转矩阵转换为末端位姿
v_=Q(:,3);
v=d_7*v_;
w=p;%-[0 0 d_1].';
u=w-v;
%% 用余弦定理计算中间关节变量
a=norm(u);
COSthe=(a_3^2+a_4^2-a^2)/(2*a_3*a_4);
medjoiang=acos(COSthe);
end

