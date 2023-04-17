function J = wholevelocityJacobian(L,rr,R_B5,T,uj,vj,wj,ud,vd,wd)
%wholeJacobian 矢量积计算仿生机械臂的速度雅克比矩阵
%   输入：当前并联机器人的向量状态
%   输出：雅克比矩阵
%% 肩关节
p_BT=T(1:3,4);
Jaj=RRR3Jacobian(uj,vj,wj);
Jlj=-skew(p_BT)*Jaj;
%% 肘关节
p_4T=p_BT-rr(1:3,4);
% R_5_5=rotz(pi/2)*rotx(pi/2);
% R_B4=R_B5*(R_5_5.');
% p_BT4=R_B4*p_4T;
Jl4=cross(L(4:6,4),p_4T);
Ja4=L(4:6,4);
%% 腕关节
Jd=RRR3Jacobian(ud,vd,wd);
p_7T=p_BT-rr(:,7);
% R_BT=T(1:3,1:3);
% p_BT7=R_BT*p_7T;
Jld=-skew(p_7T)*R_B5*Jd;
Jad=R_B5*Jd;
%% 整合
% J=[     Jl1 Jl2 Jl3 Jl4 Jl5 Jl6 Jl7;     Ja1 Ja2 Ja3 Ja4 Ja5 Ja6 Ja7 ];
J=[
    Jlj Jl4 Jld;
    Jaj Ja4 Jad
];
end