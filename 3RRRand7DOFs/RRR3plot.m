function [u,v,w,psi] = RRR3plot()
%% RRR3inverse 3RRR并联机器人初始位置画图程序
% 输入：
% theta：定平台连接三个关节的关节角度
% 输出：
% u：定平台的转动副轴线
% v：中间关节的转动副轴线
% w：动平台的转动副轴线
% psi：中间关节的角度
%% 结构参数
[gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig();
theta=[0 0 0].';
% gamma=50/180*pi;
% beta=50/180*pi;
% alpha_1=pi/2;
% % alpha_2=pi/2;
% eta=[0 2/3*pi 4/3*pi].';
%% u向量
% u_i：先绕z轴转(pi/2+eta),再绕自身y轴转(pi-gamma)
% 再绕自身z轴转(pi/2)，到达基座旋转关节初始位置
% eta=(i-1)*2/3*pi,i=1,2,3
Q_u=cell(1,3);
u=ones(3);
for i=1:3
    Q_u{i}=rotz(pi/2+eta(i))*roty(pi-gamma)*floor(rotz(pi/2));
    u_0=[0 0 1].';
    u(:,i)=Q_u{i}*u_0;
end
%% v向量：中间旋转接头
% 基座旋转初始关节u_i绕自身z轴旋转theta,再绕y轴旋转结构参数alpha_1,从而得到v_i
% v_i:u_i绕基座关节坐标系z轴旋转theta,再绕自身坐标系y轴转alpha_1
% Q_v=Q_u*rotz(theta+pi/2)*roty(alpha_1);%为什么要加一个pi/2？？？
Q_v=cell(1,3);
v=ones(3);
for i=1:3
    Q_v{i}=Q_u{i}*rotz(theta(i))*roty(alpha_1);
    v_0=[0 0 1].';
    v(:,i)=Q_v{i}*v_0;
end
%% w向量
% 当移动平台MP到达初始位置时，
% w_i：先绕z轴转(pi/2+eta_i),再绕自身y轴转beta
% 再绕自身z轴转(-pi/2)，到达基座旋转关节初始位置
% eta_i=(i-1)*2/3*pi,i=1,2,3
Q_w=cell(1,3);
w=ones(3);
for i=1:3
    Q_w{i}=rotz(pi/2+eta(i))*roty(beta)*floor(rotz(-pi/2));
    w_0=[0 0 1].';
    w(:,i)=Q_w{i}*w_0;
end

%% 求初始psi
% 垂直向量
uv=cross(v,u);
vw=cross(w,v);
% 之前求错了，应该求二面角！！！
% 用方向向量法
psi=ones(3,1);

for i=1:3
    psi(i)=acos(dot(uv(:,i),vw(:,i))/(norm(uv(:,i))*norm(vw(:,i))));
end
%% 画图
hold on
view([6,2,2]);
limit=[-1,1];xlim(limit);ylim(limit);zlim(limit);
plot3(0,0,0,'yo','MarkerFaceColor','k')
for i=1:3
    plot3(u(1,i),u(2,i),u(3,i),'yo','MarkerFaceColor','y')
    plot3(v(1,i),v(2,i),v(3,i),'yo','MarkerFaceColor','r')
    plot3(w(1,i),w(2,i),w(3,i),'yo','MarkerFaceColor','g')
    
    plot3([0,u(1,i)],[0,u(2,i)],[0,u(3,i)],'b--','MarkerFaceColor','y')
    plot3([u(1,i),v(1,i)],[u(2,i),v(2,i)],[u(3,i),v(3,i)],'b-','MarkerFaceColor','r')
    plot3([v(1,i),w(1,i)],[v(2,i),w(2,i)],[v(3,i),w(3,i)],'b-','MarkerFaceColor','r')
end


end

