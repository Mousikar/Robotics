function [theta,psi,u,v,w] = RRR3inverse(phi,err)
%% RRR3inverse 3RRR并联机器人反解程序
% 输入：
% phi：动平台位姿z-y-z欧拉角度
% 输出：theta,psi
% theta：定平台关节角
% psi：中间关节角
%% 结构参数
    if nargin==1
        err=[0 0 0 0 0 0 0 0 0 0 0 0].';
    end
[gamma,beta,alpha_1,alpha_2,eta,~,~,~] = paraconfig(err);
%% 反解程序
% w向量
% 当移动平台MP到达初始位置时，
% w_i：先绕p轴转(pi/2+eta_i),再绕自身y轴转beta
% 再绕自身z轴转(-pi/2)，到达基座旋转关节初始位置
% eta_i=(i-1)*2/3*pi,i=1,2,3
Q_w=cell(1,3);
w=ones(3);
Q=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
w_0=[0 0 1].';
p_0=[0 0 1].';
p=Q*p_0;
theta1=ones(2,3);
theta=ones(3,1);
for i=1:3
    % w向量
    % 当移动平台MP到达初始位置时，
    % w_i：先绕p轴转(pi/2+eta_i),再绕自身y轴转beta
    % 再绕自身z轴转(-pi/2)，到达基座旋转关节初始位置
    % eta_i=(i-1)*2/3*pi,i=1,2,3
    Q_w{i}=Q*rotz(pi/2+eta(i))*roty(beta(i))*floor(rotz(-pi/2));
    w(:,i)=Q_w{i}*w_0;
    % 反解theta
    % 求U,V,W
    w_x=w(1,i);
    w_y=w(2,i);
    w_z=w(3,i);
    U=-sin(alpha_1(i))*sin(eta(i))*w_y-sin(alpha_1(i))*cos(eta(i))*w_x;
    V=sin(alpha_1(i))*cos(gamma(i))*cos(eta(i))*w_y-sin(alpha_1(i))*cos(gamma(i))*sin(eta(i))*w_x+sin(alpha_1(i))*sin(gamma(i))*w_z;
    W=w_y*cos(alpha_1(i))*sin(gamma(i))*cos(eta(i))-w_x*cos(alpha_1(i))*sin(eta(i))*sin(gamma(i))-w_z*cos(alpha_1(i))*cos(gamma(i))-cos(alpha_2(i));
    % 求A,B,C
    A=W-U;
    B=2*V;
    C=W+U;
    sol=[-B+sqrt(B^2-4*A*C);-B-sqrt(B^2-4*A*C)];
    theta1(:,i)=2*atan2(sol,2*A);
    if theta1(1,i)>pi
        theta(i)=theta1(1,i)-2*pi;
    elseif theta1(1,i)<-pi
        theta(i)=theta1(1,i)+2*pi;
    else
        theta(i)=theta1(1,i);
    end
end
%% u向量
% u_i：先绕z轴转(pi/2+eta),再绕自身y轴转(pi-gamma)
% 再绕自身z轴转(pi/2)，到达基座旋转关节初始位置
% eta=(i-1)*2/3*pi,i=1,2,3
Q_u=cell(1,3);
u=ones(3);
for i=1:3
    Q_u{i}=rotz(pi/2+eta(i))*roty(pi-gamma(i))*floor(rotz(pi/2));
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
    Q_v{i}=Q_u{i}*rotz(theta(i))*roty(alpha_1(i));
    v_0=[0 0 1].';
    v(:,i)=Q_v{i}*v_0;
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
% hold on
% view([6,2,2]);
% limit=[-1,1];xlim(limit);ylim(limit);zlim(limit);
% plot3(0,0,0,'yo','MarkerFaceColor','k')
% plot3(p(1),p(2),p(3),'yo','MarkerFaceColor','k')
% for i=1:3
%     plot3(u(1,i),u(2,i),u(3,i),'yo','MarkerFaceColor','b')
%     plot3(v(1,i),v(2,i),v(3,i),'yo','MarkerFaceColor','r')
%     plot3(w(1,i),w(2,i),w(3,i),'yo','MarkerFaceColor','g')  
%     
%     plot3([0,u(1,i)],[0,u(2,i)],[0,u(3,i)],'b--')
%     plot3([0,w(1,i)],[0,w(2,i)],[0,w(3,i)],'k--')
%     plot3([u(1,i),u(1,ceil(mod(i+1,3.1)))],[u(2,i),u(2,ceil(mod(i+1,3.1)))],[u(3,i),u(3,ceil(mod(i+1,3.1)))],'b--')    
%     plot3([w(1,i),w(1,ceil(mod(i+1,3.1)))],[w(2,i),w(2,ceil(mod(i+1,3.1)))],[w(3,i),w(3,ceil(mod(i+1,3.1)))],'k--')
%     
%     plot3([u(1,i),v(1,i)],[u(2,i),v(2,i)],[u(3,i),v(3,i)],'r-')
%     plot3([v(1,i),w(1,i)],[v(2,i),w(2,i)],[v(3,i),w(3,i)],'g-')
% end


end

