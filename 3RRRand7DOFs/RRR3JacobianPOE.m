function J = RRR3JacobianPOE(u,v,w)
%RRR3JacobianPOE 并联机器人的空间雅克比矩阵
%   输入：当前并联机器人的向量状态
%   输出：雅克比矩阵
J1=[u(:,1),v(:,1),w(:,1)];
J2=[u(:,2),v(:,2),w(:,2)];
J3=[u(:,3),v(:,3),w(:,3)];
Ha=[J1(:,1),-J2(:,1),zeros(3,1);zeros(3,1),-J2(:,1),J3(:,1)];
Hp=[J1(:,2),J1(:,3),-J2(:,2),-J2(:,3),zeros(3,1),zeros(3,1);
    zeros(3,1),zeros(3,1),-J2(:,2),-J2(:,3),J3(:,2),J3(:,3)];
G=-Hp\Ha;
temp=[[1 0 0];G(1,:);G(2,:)];
J=J1*temp;
end