function qq = wholePOEinverse(T,err)
%wholePOEforward 整体机器人指数积逆解
%   输出：7个关节角
%   输入：末端执行器位姿
%% 参数
qq=[0 0 0 0 0 0 0].';
    if nargin==1
        err=[0 0 0 0 0 0 0 0 0 0 0 0].';
    end
[gamma,beta,alpha_1,~,eta,a_3,a_4,d_7] = paraconfig(err);
theta1=[0 0 0].';
%% u向量
Q_u=cell(1,3);
u=ones(3);
for i=1:3
    Q_u{i}=rotz(pi/2+eta(i))*roty(pi-gamma(i))*floor(rotz(pi/2));
    u_0=[0 0 1].';
    u(:,i)=Q_u{i}*u_0;
end
%% v向量：中间旋转接头
Q_v=cell(1,3);
v=ones(3);
for i=1:3
    Q_v{i}=Q_u{i}*rotz(theta1(i))*roty(alpha_1(i));
    v_0=[0 0 1].';
    v(:,i)=Q_v{i}*v_0;
end
%% w向量
Q_w=cell(1,3);
w=ones(3);
for i=1:3
    Q_w{i}=rotz(pi/2+eta(i))*roty(beta(i))*floor(rotz(-pi/2));
    w_0=[0 0 1].';
    w(:,i)=Q_w{i}*w_0;
end
u_i=u(:,1);
v_i=v(:,1);
w_i=w(:,1);
p_0=[0;0;1];
%% 各个关节轴线的线矢量
r=[
    0   0   0   0       0           0           0
    0   0   0   0       0           0           0
    0   0   0   a_3     a_3+a_4     a_3+a_4     a_3+a_4
    ];
mu=[0 1 0].';
L=zeros(6,7);
L(:,1)=[cross(r(:,1),u_i); u_i];
L(:,2)=[cross(r(:,2),v_i); v_i];
L(:,3)=[cross(r(:,3),w_i); w_i];
L(:,4)=[cross(r(:,4),mu);  mu];
L(:,5)=[cross(r(:,5),u_i); u_i];
L(:,6)=[cross(r(:,6),v_i); v_i];
L(:,7)=[cross(r(:,7),w_i); w_i];

%% 初始位姿
M=[
    1 0 0 0
    0 1 0 0 
    0 0 1 a_3+a_4+d_7
    0 0 0 1
    ];
%% 计算肘关节qq(4)
    tau2pi=359;
    squa_his=zeros(tau2pi*2,1);
    qq_his=zeros(7,tau2pi*2);
for j=1:2
    T1=T*inv(M);
    % pw是腕部三轴的交点
    pw=[0 0 a_3+a_4 1].';%T*[0 0 -d_7 1].';
    pb=[0 0 0 1].';
    temp=(T1*pw-pb);
    p=pw(1:3);q=pb(1:3);delta=norm(temp(1:3));
    qq(4) = subproblem3(p,q,mu,delta,r(:,4),(2*j-3));
    %% 计算前三个关节
    for i=0:tau2pi
        %% 选择一个qq(3)
        qq(3)=-pi+i/180*pi;
        %% 计算qq(1)和qq(2)
        p=POE(L(:,3),qq(3))*POE(L(:,4),qq(4))*pw;
        q=T1*pw;
        try
            [qq(1),qq(2)] = subproblem2(p(1:3),q(1:3),u_i,v_i,r(:,1),1);
            %% 求腕关节qq(5)qq(6)
            % p在L7上，但不在轴线L5L6上
            p=[r(:,7)+100*w_i;1];
            T2=POE(L(:,4),-qq(4))*POE(L(:,3),-qq(3))*POE(L(:,2),-qq(2))*POE(L(:,1),-qq(1))*T*inv(M);
            q=T2*p;
            [qq(5),qq(6)] = subproblem2(p(1:3),q(1:3),u_i,v_i,r(:,7),1);
            %% 求腕关节qq(7)
            % p为L7轴线外的一点
            p=[r(:,7)+10*w_i+[0 0 1].'; 1];
            T3=POE(-L(:,6),qq(6))*POE(-L(:,5),qq(5))*POE(-L(:,4),qq(4))*POE(-L(:,3),qq(3))*POE(-L(:,2),qq(2))*POE(-L(:,1),qq(1))*T*inv(M);
            q=T3*p;
            qq(7) = subproblem1(p(1:3),q(1:3),w_i,r(:,7));
            %% 储存关节角
            squa_his((j-1)*(tau2pi+1)+i+1)=norm(qq)^2;
            qq_his(:,(j-1)*(tau2pi+1)+i+1)=qq;
        catch
            squa_his((j-1)*(tau2pi+1)+i+1)=2^32-1;
            qq_his(:,(j-1)*(tau2pi+1)+i+1)=2^32-1;
        end

    end

end
qq=qq_his(:,squa_his==min(squa_his));

%% 可视化
% % % figure
% view([600,50,50]);
% hold on
% plot3(0,0,0,'bo','MarkerFaceColor','b')
% % [R,pj,~,~,~] = RRR3expprod(qq(1),qq(2),qq(3));
% TT1=POE(L(:,1),qq(1))*POE(L(:,2),qq(2))*POE(L(:,3),qq(3));
% a3=TT1*[0 0 a_3 1].';
% plot3([0,a3(1)],[0,a3(2)],[0,a3(3)],'b-')
% plot3(a3(1),a3(2),a3(3),'bo','MarkerFaceColor','b')
% % pd=R*rotx(-pi/2)*rotz(-pi/2)*rotz(qq(4))*[1 0 0].';
% % m=T*[0 0 -d_7 1].';
% TT2=TT1*POE(L(:,4),qq(4));
% a4=TT2*[0 0 a_3+a_4 1].';
% TT3=TT2*POE(L(:,5),qq(5))*POE(L(:,6),qq(6))*POE(L(:,7),qq(7));
% p_=TT3*[0 0 a_3+a_4+d_7 1].';
% plot3([a3(1),a4(1)],[a3(2),a4(2)],[a3(3),a4(3)],'b-')
% plot3(a4(1),a4(2),a4(3),'bo','MarkerFaceColor','b')
% % p_=a4+d_7*T(1:3,3);
% plot3([p_(1),a4(1)],[p_(2),a4(2)],[p_(3),a4(3)],'b-')
% plot3(p_(1),p_(2),p_(3),'ro','MarkerFaceColor','r')
% axis equal
%         view([600,50,50]);
%         hold on
%         plot3([0,0],[0,0],[0,0],'b-')
%         plot3(0,0,0,'bo','MarkerFaceColor','b')
%         [~,pj,~,~,~] = RRR3expprod(q(1),q(2),q(3));
%         a3=[0 0 0].'+a_3*pj;
%         plot3([0,a3(1)],[0,a3(2)],[0,a3(3)],'b-')
%         plot3(a3(1),a3(2),a3(3),'bo','MarkerFaceColor','b')
%         pd=m-a_3*pj;
%         a4=a3+pd;
%         plot3([a3(1),a4(1)],[a3(2),a4(2)],[a3(3),a4(3)],'b-')
%         plot3(a4(1),a4(2),a4(3),'bo','MarkerFaceColor','b')
%         p_=a4+d_7*T_BT(1:3,3);
%         plot3([p_(1),a4(1)],[p_(2),a4(2)],[p_(3),a4(3)],'b-')
%         plot3(p_(1),p_(2),p_(3),'ro','MarkerFaceColor','r')
%         axis equal
end

