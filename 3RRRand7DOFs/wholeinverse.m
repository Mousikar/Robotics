function [theta_j,psi_j,theta_d,psi_d,medjoiang] = wholeinverse(p,phi)
%wholeinverse 整体机械臂反解
%   输入：末端位姿
%   输出：各个关节角
%% 参数
% p=[10 5 520].';
% phi=[pi/9 5*pi/10 pi/12].';
% p=[150 160 170].';
% phi=[pi/4 5*pi/6 pi/6].';
p=[200 210 220].';
phi=[pi/4 5*pi/6 pi/6].';
% p=[0 0 535].';
% phi=[0 0 0 ].';
[gamma,beta,alpha_1,alpha_2,eta,a_3,a_4,d_1] = paraconfig();
%% 求解
[medjoiang,u] = solveMedJoiAng(p,phi);
[pj,pd] = choosepj(u);
% 中间关节的坐标系定义
zr=cross(pd,pj);zr=zr/norm(zr);
xr=pd;
yr=cross(zr,xr);yr=yr/norm(yr);
Qr=[xr yr zr];
% 静平台坐标系定义
% zj=xr;
% yj=-yr;
% xj=zr;
zj=yr;
yj=xr;
xj=zr;
Qj=[xj yj zj];
% 静平台的坐标系定义
yd=zr;
xd=yr;
zd=xr;
Qd=[xd yd zd];
%% 静平台关节变量求解
[theta_j,psi_j] = RRR3inversePro(Qj);

%% 动平台关节变量求解
% 将末端位姿转化为旋转矩阵
Q=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
QD=Qd.'*Q;
[theta_d,psi_d] = RRR3inverse(QD);
% %% 静平台关节变量求解
% ang_j=zeros(3,1);
% [ang_j(1),ang_j(2),ang_j(3)] = r2zyz(Qj);
%     logi=Qj-rotz(ang_j(1))*roty(ang_j(2))*rotz(ang_j(3))<=0.0001;
%     if sum(sum(logi))~=9
%         disp('wrong')
%     end
% [theta_j,psi_j] = RRR3inverse(ang_j);
% 
% %% 动平台关节变量求解
% % 将末端位姿转化为旋转矩阵
% Q=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
% QD=Qd.'*Q;
% ang_d=zeros(3,1);
% [ang_d(1),ang_d(2),ang_d(3)] = r2zyz(QD);
%     logi=Q-rotz(ang_d(1))*roty(ang_d(2))*rotz(ang_d(3))<=0.0001;
%     if sum(sum(logi))~=9
%         disp('wrong')
%     end
% [theta_d,psi_d] = RRR3inverse(ang_d);
%% 可视化
figure
hold on
plot3([0,0],[0,0],[0,d_1],'b-')
plot3(0,0,d_1,'bo','MarkerFaceColor','b')
a3=[0 0 d_1].'+a_3*pj;
plot3([0,a3(1)],[0,a3(2)],[d_1,a3(3)],'b-')
plot3(a3(1),a3(2),a3(3),'bo','MarkerFaceColor','b')
a4=a3+a_4*pd;
plot3([a3(1),a4(1)],[a3(2),a4(2)],[a3(3),a4(3)],'b-')
plot3(a4(1),a4(2),a4(3),'bo','MarkerFaceColor','b')
p_=a4+d_1*Q(:,3);
plot3([p_(1),a4(1)],[p_(2),a4(2)],[p_(3),a4(3)],'b-')
plot3(p_(1),p_(2),p_(3),'bo','MarkerFaceColor','b')
axis equal
end

