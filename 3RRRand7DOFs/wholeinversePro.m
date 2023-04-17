function q = wholeinversePro(T_BT)
%wholeinverse 整体机械臂反解
%   输入：末端工具位姿
%   输出：各个关节角
%% 参数
[gamma,beta,alpha_1,alpha_2,eta,a_3,a_4,d_7] = paraconfig();
q=[0 0 0 0 0 0 0].';

        
        
%% 求解T_B3
[q(4),m] = solveMedJoiAngPro(T_BT);
a=norm(m);
n=d_7*T_BT(1:3,3);
sigma=acos((a_3^2+a^2-a_4^2)/(2*a_3*a));
tau2pi=359;
squa_his=zeros(tau2pi*2,1);
q_his=zeros(7,tau2pi*2);
for i=0:tau2pi
    tau=i/180*pi;
    p_Mj=rotz(tau)*rotx(sigma)*[0 0 1].';
    R_BM=diag([1 1 1]);
    R_BM(1:3,3)=m/a;
    R_BM(1:3,2)=cross([m(1) m(2) 0].',m)/norm(cross([m(1) m(2) 0].',m));
    R_BM(1:3,1)=cross(R_BM(1:3,2),R_BM(1:3,3));
    p_Bj=R_BM*p_Mj;   
    
    T_B3=diag([1 1 1 1]);
    T_B3(1:3,3)=p_Bj;
    for j=1:2
        p_j=a_3*p_Bj;
        p_d=m-p_j;
        T_B3(1:3,2)=(2*j-3)*cross(p_j,p_d)/norm(cross(p_j,p_d));
        T_B3(1:3,1)=cross(T_B3(1:3,2),T_B3(1:3,3));
        %%
        T_3_4=transformation(a_3,0,0,q(4));%screwx(a_3,-pi/2)*screwz(0,q(4));
        T_45_=transformation(a_4,0,0,0);

        T_33_=Rotx(-pi/2)*Rotz(-pi/2);
        T_5_5=Rotz(pi/2)*Rotx(pi/2);

        T_WT=transformation(0,0,d_7,0);%screwx(0,0)*screwz(d_7,0);
        T_5W=inv(T_5_5)*inv(T_45_)*inv(T_3_4)*inv(T_33_)*inv(T_B3)*T_BT*inv(T_WT);
        try
        [q(1),q(2),q(3)] = RRR3expprodinv(T_B3(1:3,1:3));
        [q(5),q(6),q(7)] = RRR3expprodinv(T_5W(1:3,1:3));
        squa_his(i*2+j)=norm(q)^2;
        q_his(:,i*2+j)=q;
        catch
            squa_his(i*2+j)=2^32-1;
            q_his(:,i*2+j)=2^32-1;
        end
    end
end
q=q_his(:,squa_his==min(squa_his));
%% 可视化
% % figure
% view([600,50,50]);
% hold on
% plot3([0,0],[0,0],[0,0],'b-')
% plot3(0,0,0,'bo','MarkerFaceColor','b')
% [~,pj,~,~,~] = RRR3expprod(q(1),q(2),q(3));
% a3=[0 0 0].'+a_3*pj;
% plot3([0,a3(1)],[0,a3(2)],[0,a3(3)],'b-')
% plot3(a3(1),a3(2),a3(3),'bo','MarkerFaceColor','b')
% pd=m-a_3*pj;
% a4=a3+pd;
% plot3([a3(1),a4(1)],[a3(2),a4(2)],[a3(3),a4(3)],'b-')
% plot3(a4(1),a4(2),a4(3),'bo','MarkerFaceColor','b')
% p_=a4+d_7*T_BT(1:3,3);
% plot3([p_(1),a4(1)],[p_(2),a4(2)],[p_(3),a4(3)],'b-')
% plot3(p_(1),p_(2),p_(3),'ro','MarkerFaceColor','r')
% axis equal

% view([600,50,50]);
% hold on
% plot3(0,0,0,'bo','MarkerFaceColor','b')
% [R,pj,~,~,~] = RRR3expprod(q(1),q(2),q(3));
% a3=a_3*pj;
% plot3([0,a3(1)],[0,a3(2)],[0,a3(3)],'b-')
% plot3(a3(1),a3(2),a3(3),'bo','MarkerFaceColor','b')
% pd=R*rotx(-pi/2)*rotz(-pi/2)*rotz(q(4))*[1 0 0].';
% % m=T*[0 0 -d_7 1].';
% 
% a4=a3+a_4*pd;
% p_=a4+T_BT(1:3,4)/norm(T_BT(1:3,4))*d_7;
% plot3([a3(1),a4(1)],[a3(2),a4(2)],[a3(3),a4(3)],'b-')
% plot3(a4(1),a4(2),a4(3),'bo','MarkerFaceColor','b')
% % p_=a4+d_7*T(1:3,3);
% plot3([p_(1),a4(1)],[p_(2),a4(2)],[p_(3),a4(3)],'b-')
% plot3(p_(1),p_(2),p_(3),'ro','MarkerFaceColor','r')
% % axis equal
end

%% 函数区
function result=transformation(a,alpha,d,theta)
    result=screwx(a,alpha)*screwz(d,theta);
end
% 编写screw函数
function result=screwx(a,alpha)
    result=Rotx(alpha)*transl([a 0 0]);
end
function result=screwz(d,theta)
    result=Rotz(theta)*transl([0 0 d]);
end
% R4*4的旋转和移动矩阵
function result=Rotz(theta)
    result=[rot('z',theta) [0 0 0].';[0 0 0 1]];
end
function result=Roty(theta)
    result=[rot('y',theta) [0 0 0].';[0 0 0 1]];
end
function result=Rotx(theta)
    result=[rot('x',theta) [0 0 0].';[0 0 0 1]];
end
% 旋转矩阵
function result=rot(axis,theta)
    switch(axis)
        case 'x'
            result=[1 0 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
        case 'y'
            result=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];       
        case 'z'
            result=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0; 0 0 1];
        otherwise
            disp('wrong')
    end
    if mod(theta,pi/2)==0
        result=round(result);
    end
end

