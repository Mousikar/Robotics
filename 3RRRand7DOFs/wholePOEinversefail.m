function qq = wholePOEinverseProfail(T)
%wholePOEforward 整体机器人指数积正解
%   输出：7个关节角,前三个关节角和后三个关节角是三条支链的关节角
%   输入：末端执行器位姿
%% 参数
qq=[0 0 0 0 0 0 0].';
[gamma,beta,alpha_1,alpha_2,eta,a_3,a_4,d_7] = paraconfig();
theta1=[0 0 0].';
%% u向量
Q_u=cell(1,3);
u=ones(3);
for i=1:3
    Q_u{i}=rotz(pi/2+eta(i))*roty(pi-gamma)*floor(rotz(pi/2));
    u_0=[0 0 1].';
    u(:,i)=Q_u{i}*u_0;
end
%% v向量：中间旋转接头
Q_v=cell(1,3);
v=ones(3);
for i=1:3
    Q_v{i}=Q_u{i}*rotz(theta1(i))*roty(alpha_1);
    v_0=[0 0 1].';
    v(:,i)=Q_v{i}*v_0;
end
%% w向量
Q_w=cell(1,3);
w=ones(3);
for i=1:3
    Q_w{i}=rotz(pi/2+eta(i))*roty(beta)*floor(rotz(-pi/2));
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
L=setL(u_i,v_i,w_i,r,mu);
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
    TTT_his=cell(1,1);
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
        %% 选择一个xi
        xi=-pi+i/180*pi;
        %% 计算qq(1)和qq(2)和qq(3)
        p=POE(L(:,3),xi)*POE(L(:,4),qq(4))*pw;
        q=T1*pw;
        try
            for k=1:3
                u_i=u(:,k);
                v_i=v(:,k);
                w_i=w(:,k);
                for l=1:2
                    [theta(k,l),psit(k,l)] = subproblem2(p(1:3),q(1:3),u_i,v_i,r(:,1),l*2-3);
                    %% 最后一个关节角为子问题1
                    if k==1
                        mu_=expprod(u_i,theta(k,l))*expprod(v_i,psit(k,l))*expprod(w_i,xi)*mu;
                        r4_=expprod(u_i,theta(k,l))*expprod(v_i,psit(k,l))*expprod(w_i,xi)*r(:,4);
                        Ttem=[cross(mu_,r4_)/norm(cross(mu_,r4_)) mu_ r4_/norm(r4_)];
                    end
                    exp3=expprod(-v_i,psit(k,l))*expprod(-u_i,theta(k,l))*Ttem;
                    p1=[0 0 1].'+w_i;
                    q1=exp3*p1;
                    xitemp(k,l)=subproblem1(p1,q1,w_i);
                end
            end
            %% 选择一组合适的qq
            cou=0;
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(1)=theta(1,1);
                qq(2)=theta(2,1);
                qq(3)=theta(3,1);
                psi(1)=psit(1,1);
                psi(2)=psit(2,1);
                psi(3)=psit(3,1); 
                xit(1)=xitemp(1,1);
                xit(2)=xitemp(2,1);
                xit(3)=xitemp(3,1);
                cou=cou+1;
            end
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                    theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(1)=theta(1,1);
                qq(2)=theta(2,1);
                qq(3)=theta(3,2);
                psi(1)=psit(1,1);
                psi(2)=psit(2,1);
                psi(3)=psit(3,2);
                xit(1)=xitemp(1,1);
                xit(2)=xitemp(2,1);
                xit(3)=xitemp(3,2);
                cou=cou+1;
            end
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(1)=theta(1,1);
                qq(2)=theta(2,2);
                qq(3)=theta(3,1);
                psi(1)=psit(1,1);
                psi(2)=psit(2,2);
                psi(3)=psit(3,1);
                xit(1)=xitemp(1,1);
                xit(2)=xitemp(2,2);
                xit(3)=xitemp(3,1);
                cou=cou+1;
            end
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(1)=theta(1,1);
                qq(2)=theta(2,2);
                qq(3)=theta(3,2);
                psi(1)=psit(1,1);
                psi(2)=psit(2,2);
                psi(3)=psit(3,2);
                xit(1)=xitemp(1,1);
                xit(2)=xitemp(2,2);
                xit(3)=xitemp(3,2);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                    theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(1)=theta(1,2);
                qq(2)=theta(2,1);
                qq(3)=theta(3,1);
                psi(1)=psit(1,2);
                psi(2)=psit(2,1);
                psi(3)=psit(3,1);
                xit(1)=xitemp(1,2);
                xit(2)=xitemp(2,1);
                xit(3)=xitemp(3,1);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(1)=theta(1,2);
                qq(2)=theta(2,1);
                qq(3)=theta(3,2);
                psi(1)=psit(1,2);
                psi(2)=psit(2,1);
                psi(3)=psit(3,2);
                xit(1)=xitemp(1,2);
                xit(2)=xitemp(2,1);
                xit(3)=xitemp(3,2);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(1)=theta(1,2);
                qq(2)=theta(2,2);
                qq(3)=theta(3,1);
                psi(1)=psit(1,2);
                psi(2)=psit(2,2);
                psi(3)=psit(3,1);
                xit(1)=xitemp(1,2);
                xit(2)=xitemp(2,2);
                xit(3)=xitemp(3,1);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(1)=theta(1,2);
                qq(2)=theta(2,2);
                qq(3)=theta(3,2);
                psi(1)=psit(1,2);
                psi(2)=psit(2,2);
                psi(3)=psit(3,2);
                xit(1)=xitemp(1,2);
                xit(2)=xitemp(2,2);
                xit(3)=xitemp(3,2);
                cou=cou+1;
            end
            if cou==0                
                qq(1)=2^32-1;
                qq(2)=2^32-1;
                qq(3)=2^32-1;
            end
            %% 求腕关节qq(5)qq(6)qq(7)
            for k=1:3
                u_i=u(:,k);
                v_i=v(:,k);
                w_i=w(:,k);
                L=setL(u_i,v_i,w_i,r,mu);
                % p在L7上，但不在轴线L5L6上
                p=[r(:,7)+100*w_i;1];
                T2=POE(L(:,4),-qq(4))*POE(L(:,3),-xit(k))*POE(L(:,2),-psi(k))*POE(L(:,1),-qq(k))*T*inv(M);
                q=T2*p;
                %%
                for l=1:2
                    [theta(k,l),~] = subproblem2(p(1:3),q(1:3),u_i,v_i,r(:,7),l*2-3);
                end
            end
            %% 筛选
            cou=0;
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(5)=theta(1,1);
                qq(6)=theta(2,1);
                qq(7)=theta(3,1);
                cou=cou+1;
            end
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                    theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(5)=theta(1,1);
                qq(6)=theta(2,1);
                qq(7)=theta(3,2);
                cou=cou+1;
            end
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(1)=theta(1,1);
                qq(2)=theta(2,2);
                qq(3)=theta(3,1);
                cou=cou+1;
            end
            if theta(1,1)>=-pi/3&&theta(1,1)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(5)=theta(1,1);
                qq(6)=theta(2,2);
                qq(7)=theta(3,2);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                    theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(5)=theta(1,2);
                qq(6)=theta(2,1);
                qq(7)=theta(3,1);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,1)>=-pi/3&&theta(2,1)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(5)=theta(1,2);
                qq(6)=theta(2,1);
                qq(7)=theta(3,2);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,1)>=-pi/3&&theta(3,1)<=pi/2
                qq(5)=theta(1,2);
                qq(6)=theta(2,2);
                qq(7)=theta(3,1);
                cou=cou+1;
            end
            if theta(1,2)>=-pi/3&&theta(1,2)<=pi/2&&...
                    theta(2,2)>=-pi/3&&theta(2,2)<=pi/2&&...
                    theta(3,2)>=-pi/3&&theta(3,2)<=pi/2
                qq(5)=theta(1,2);
                qq(6)=theta(2,2);
                qq(7)=theta(3,2);
                cou=cou+1;
            end
            if cou==0                
                qq(5)=2^32-1;
                qq(6)=2^32-1;
                qq(7)=2^32-1;
            end
            %% 储存关节角
            squa_his((j-1)*(tau2pi+1)+i+1)=norm(qq)^2;
            qq_his(:,(j-1)*(tau2pi+1)+i+1)=qq;
            TTT_his{(j-1)*(tau2pi+1)+i+1}=Ttem;
        catch
            squa_his((j-1)*(tau2pi+1)+i+1)=2^32-1;
            qq_his(:,(j-1)*(tau2pi+1)+i+1)=2^32-1;
        end

    end

end
% qq=qq_his(:,squa_his==min(squa_his));
qq=qq_his(:,find(squa_his<2^32-1));
qq=qq(:,find(sum(qq.*qq,1)==min(sum(qq.*qq,1))));

% %% 可视化
% % % % figure
% TT1=TTT_his{squa_his==min(squa_his)};
% view([600,50,50]);
% hold on
% plot3(0,0,0,'bo','MarkerFaceColor','b')
% % %%
% % u_i=u(:,1);
% % v_i=v(:,1);
% % w_i=w(:,1);
% % L=setL(u_i,v_i,w_i,r,mu);
% % % RR1=expprod(u_i,qq(3))*expprod(v_i,psi(3))*expprod(w_i,xit(3));
% % TT1=POE(L(:,1),qq(1))*POE(L(:,2),psi(1))*POE(L(:,3),xit(1));
% %%
% a3=[TT1,[0 0 0].';[0 0 0 1]]*[0 0 a_3 1].';
% plot3([0,a3(1)],[0,a3(2)],[0,a3(3)],'b-')
% plot3(a3(1),a3(2),a3(3),'bo','MarkerFaceColor','b')
% %%
% p_=T(:,4);
% a4=p_-d_7*T(:,3);
% plot3([a3(1),a4(1)],[a3(2),a4(2)],[a3(3),a4(3)],'b-')
% plot3(a4(1),a4(2),a4(3),'bo','MarkerFaceColor','b')
% plot3([p_(1),a4(1)],[p_(2),a4(2)],[p_(3),a4(3)],'b-')
% plot3(p_(1),p_(2),p_(3),'ro','MarkerFaceColor','r')
% disp(norm(a3(1:3)-a4(1:3)))
% axis equal
end

% 设定线矢量
function L=setL(u_i,v_i,w_i,r,mu)
    L=zeros(6,7);
    L(:,1)=[cross(r(:,1),u_i); u_i];
    L(:,2)=[cross(r(:,2),v_i); v_i];
    L(:,3)=[cross(r(:,3),w_i); w_i];
    L(:,4)=[cross(r(:,4),mu);  mu];
    L(:,5)=[cross(r(:,5),u_i); u_i];
    L(:,6)=[cross(r(:,6),v_i); v_i];
    L(:,7)=[cross(r(:,7),w_i); w_i];
end