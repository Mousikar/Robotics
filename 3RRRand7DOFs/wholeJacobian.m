function J = wholeJacobian(L,rr,R_B5,uj,vj,wj,ud,vd,wd)
%wholeJacobian 仿生机械臂的空间雅克比矩阵
%   输入：当前并联机器人的向量状态
%   输出：雅克比矩阵
%% 肩关节
% Jl1=[0,0,0].';Jl2=[0,0,0].';Jl3=[0,0,0].';
Jlj=[0 0 0;0 0 0;0 0 0];
Jaj=RRR3Jacobian(uj,vj,wj);
% Ja1=Jaj(:,1); Ja2=Jaj(:,2); Ja3=Jaj(:,3);
%% 肘关节
Jl4=L(1:3,4);%cross(L(1:3,4),L(4:6,4));
Ja4=L(4:6,4);
%% 腕关节
Jd=RRR3Jacobian(ud,vd,wd);
Jld=skew(rr(1:3,5))*R_B5*Jd;
Jad=R_B5*Jd;
%% 整合
% J=[     Jl1 Jl2 Jl3 Jl4 Jl5 Jl6 Jl7;     Ja1 Ja2 Ja3 Ja4 Ja5 Ja6 Ja7 ];
J=[
    Jlj Jl4 Jld;
    Jaj Ja4 Jad
];
end