%% 保存7个关节角为mat文件，连续的关节
clear;
count=1;
num=50;
q=[pi/8 pi/9 pi/10 pi/3 pi/11 pi/12 pi/13];
for k=1:7
    q_hitory(k,:)=linspace(0,q(k),num);
end
count=num+1;
for i=1:count-1
    time(i,1)=i/(count-1)*10;
end
for j=1:7
    qqq=q_hitory(j,:).';
    qqq=[time,qqq];
    qqq=qqq.';
    save(strcat('q',num2str(j)),'qqq');
end
%% 修正程序
clear;clc
load('T_history.mat')
T=T_history{50}
% 求运动学等效肢体反解
qq = wholePOEinverse(T)
[R1,~,~,~,~] = RRR3expprod(qq(1),qq(2),qq(3))
[R2,~,~,~,~] = RRR3expprod(qq(5),qq(6),qq(7))
[the1,psi1,xi1] = RRR3expprodinvPro(R1)
q(1:3,1)=the1(:,2);
[the2,psi2,xi2] = RRR3expprodinvPro(R2)
q(5:7,1)=the2(:,2);
q(4,1)=qq(4)
%% 单个关节角验证
q=[-0.0434
    0.0647
   -0.0175
    0.1410
    0.4627
    0.4419
    0.0098];
% 得到存在的位姿
T=wholeforward(q)
% 反解出驱动关节
qq = wholePOEinversePro(T)
% 正解出末端位姿
wholeforwardPro(qq(:,1))
%% 整体机械臂正解测试
clear
num=100;
a=linspace(0,pi/9,num);
b=linspace(0,pi/6,num);
c=linspace(0,pi/11,num);
r=linspace(0,pi/9,num);
a1=linspace(0,pi/9,num);
b1=linspace(0,pi/6,num);
c1=linspace(0,pi/11,num);
% 可行
% a=linspace(0,pi/9,num);
% b=linspace(0,pi/10,num);
% c=linspace(0,pi/11,num);
% r=linspace(pi/4,pi/2,num);
% a1=linspace(0,pi/9,num);
% b1=linspace(0,pi/10,num);
% c1=linspace(0,pi/11,num);
temp=diag([1 1 1 1]);
temp(:,4)=[0 0 538 0 ].';
T_history=cell(num,1);
T_history{1}=temp;
global theta;
theta=1;
for i=1:num
    q=[a(i) b(i) c(i) r(i) a1(i) b1(i) c1(i)];
    T_BT = wholeforwardPro(q);
    temp= T_BT;
    T_history{i}=T_BT;
end
%% 指数积模型得到可达到的位姿
clear
num=50;
a=linspace(0,pi/9,100);
b=linspace(0,pi/6,100);
c=linspace(0,pi/11,100);
r=linspace(0,pi/9,100);
a1=linspace(0,pi/9,100);
b1=linspace(0,pi/6,100);
c1=linspace(0,pi/11,100);
T_history=cell(num,1);
for i=1:num
    q=[a(i) b(i) c(i) r(i) a1(i) b1(i) c1(i)];
    T_BT = wholePOEforward(q);
    T_history{i}=T_BT;
end
%% 画图
figure   
for i=1:num
    t=T_history{i};     
    plot3(t(1,4),t(2,4),t(3,4),'ro','MarkerFaceColor','r')
    hold on;
end
T_history{i}
%% 整体机械臂指数积反解测试
T_BT_history=cell(num,1);
count=1;
qtem=0;
for i=1:num
    T_BT=T_history{i};
    qq = wholePOEinversePro(T_BT);
    siz=qq-qtem;
    q=qq(:,sum(siz.*siz,1)==min(sum(siz.*siz,1)));
    qtem=q;
    q_hitory(:,count)=q(:,1);
    count=count+1;
    T_BT_history{i} = wholeforwardPro(q);
    pause(0.1)
end
%% 保存7个关节角为mat文件
for i=1:count-1
    time(i,1)=i/(count-1)*10;
end
for j=1:7
    qqq=q_hitory(j,:).';
    qqq=[time,qqq];
    qqq=qqq.';
    save(strcat('q',num2str(j)),'qqq');
end
%% 检查反解是否正确
for i=1:length(T_history)    
    if norm(T_history{i}-T_BT_history{i})>=0.01
        disp('wrong')
    else
        disp('right')
    end
end
disp('check over')
%% 关节角plot
figure
subplot(7,1,1)
plot(q_hitory(1,:)/pi*180)
subplot(7,1,2)
plot(q_hitory(2,:)/pi*180)
subplot(7,1,3)
plot(q_hitory(3,:)/pi*180)
subplot(7,1,4)
plot(q_hitory(4,:)/pi*180)
subplot(7,1,5)
plot(q_hitory(5,:)/pi*180)
subplot(7,1,6)
plot(q_hitory(6,:)/pi*180)
subplot(7,1,7)
plot(q_hitory(7,:)/pi*180)
%% 写字
clear
load('T_sR.mat')
count=1;
qtem=0;
T_BT_history=cell(length(T),1);
for i=1:20:length(T)
    T_BT=T{i};
    qq = wholePOEinversePro(T_BT);
    siz=qq-qtem;
    q=qq(:,sum(siz.*siz,1)==min(sum(siz.*siz,1)));
    qtem=q;
    
    T_BT_history{i}=wholeforwardPro(q);
    q_hitory(:,count)=q(:,1);
    count=count+1;
end
%% 检查
clc
for i=1:20:length(T)    
    if norm(T{i}-T_BT_history{i})>=0.01
        disp('wrong')
    else
        disp('right')
    end
end
disp('check over')
%% RRR3的雅克比矩阵测试
Q=rotz(pi/12)*roty(pi/10)*rotz(pi/14);
% Q=rotz(0)*roty(0)*rotz(0);
[u,v,w] = RRR3getuvw(Q);
Jvector = RRR3Jacobian(u,v,w)
JPOE=RRR3JacobianPOE(u,v,w)
disp('----------------------------------------------')
Q=rotz(0)*roty(0)*rotz(0);
[u,v,w] = RRR3getuvw(Q);
Jvector = RRR3Jacobian(u,v,w)
JPOE=RRR3JacobianPOE(u,v,w)
Q=rotz(0)*roty(0)*rotz(0);
dottheta=[0.01 0.02 0.01].';
dotomega=Jvector*dottheta;
%% 一组位姿的误差
num=50;
alpha=linspace(0,pi/3,num);
beta=linspace(0,pi/4,num);
gamma=linspace(0,pi/10,num);
for i=1:num
    Q=rotz(alpha(i))*roty(beta(i))*rotz(gamma(i));
    [u,v,w] = RRR3getuvw(Q);
    J = RRR3Jacobian(u,v,w);
    dottheta=[0 0 0 ].';
    dotomega(:,i)=J*dottheta;
end
% plot(dotomega.')
plot1 = plot(dotomega.','LineWidth',2);
legend('\omega_x', '\omega_y','\omega_z')
%% 仿生机械臂的雅克比矩阵
% T=[
%    -0.0591    0.0100    0.9982 -144.7723
%    -0.0006   -1.0000    0.0099   -6.4208
%     0.9983         0    0.0591  161.8541
%          0         0         0    1.0000];
% 
% T =[
%     0.0000    0.0000    1.0000  283.0000
%     0.0000    1.0000   -0.0000   -0.0000
%    -1.0000    0.0000    0.0000  255.0000
%          0         0         0    1.0000];
load('T_history.mat')
T=T_history{30};
R_BT=T(1:3,1:3);p=T(1:3,4);pp=T(1:4,4);
[LL,rr,R_B5,uj,vj,wj,ud,vd,wd]  = wholegetL(T);
% T=eye(4);T(3,4)=538;
%% 空间雅可比矩阵
disp('----------------------------------------------')
J = wholeJacobian(LL,rr,R_B5,uj,vj,wj,ud,vd,wd);
% dottheta=[0.01 0.02 0.01 0.01 0.02 0.01 0.02].';
% dottheta=[0.01 0.01 0.01 0.01 0.01 0.01 0.01].';

% dottheta=[1 0 0 0 0 0 0].';
% dottheta=[0 1 0 0 0 0 0].';
% dottheta=[0 0 1 0 0 0 0].';
% dottheta=[0 0 0 1 0 0 0].';
% dottheta=[ 0 0 0 0 1 0 0].';
% dottheta=[ 0 0 0 0 0 1 0].';
% dottheta=[ 0 0 0 0 0 0 1].';
dottheta=[1 2 3 4 1 2 3].';
disp(dottheta.')
spacevomega=J*dottheta;
ppdot=skewV(spacevomega)*pp;
vomega=spacevomega;vomega(1:3)=ppdot(1:3);
disp(vomega.')
% Ad=[R_BT.' -(R_BT.')*skew(p);
%     zeros(3,3) R_BT.']
Ad=Adjointmatrix(T);
Ad=inv(Ad);
JT=Ad*J;
spacevomegaT=JT*dottheta;
disp(spacevomegaT.')
% norm(vomega(1:3))
% norm(spacevomegaT(1:3))
% norm(vomega(4:6))
% norm(spacevomegaT(4:6))
% 矢量积求解速度雅可比矩阵
JTV = wholevelocityJacobian(LL,rr,R_B5,T,uj,vj,wj,ud,vd,wd);
veloomega=JTV*dottheta;
disp(veloomega.')
JTVT=[R_BT.' zeros(3,3);zeros(3,3) R_BT.']*JTV;
veloomegaT=JTVT*dottheta;
disp(veloomegaT.')
% norm(veloomega(1:3))
% norm(veloomegaT(1:3))
% norm(veloomega(4:6))
% norm(veloomegaT(4:6))
%% 求一组误差
clear
load('T_history.mat');
T=T_history;
%% 绝对误差
for j=1:7
    %%
for i=3:length(T)
    [LL,rr,R_B5,uj,vj,wj,ud,vd,wd]  = wholegetL(T{i});
    J = wholeJacobian(LL,rr,R_B5,uj,vj,wj,ud,vd,wd);
%     dottheta=[0.01 0.01 0.01 0.01 0.01 0.01 0.01].';
%     dottheta=[0.01 0 0 0 0 0 0].';
%     dottheta=[0 0.01 0 0 0 0 0].';
%     dottheta=[0 0 0.01 0 0 0 0].';
%     dottheta=[0 0 0 0.01 0 0 0].';
%     dottheta=[0 0 0 0 0.01 0 0].';
%     dottheta=[0 0 0 0 0 0.01 0].';
%     dottheta=[0 0 0 0 0 0 0.01].';
    dottheta=zeros(7,1);
    dottheta(j)=0.01;
    spacevomega=J*dottheta;
    R_BT=T{i}(1:3,1:3);p=T{i}(1:3,4);pp=T{i}(1:4,4);
    ppdot=skewV(spacevomega)*pp;
    vomega=spacevomega;vomega(1:3)=ppdot(1:3);
    vomega_h(:,i)=vomega;
%     Ad=[R_BT.' -(R_BT.')*skew(p);
%         zeros(3,3) R_BT.'];%zeros(3,3),skew(p)*R_BT.'
%     JT=Ad*J;
%     spacevomegaT(:,i)=JT*dottheta;

%     JTV = wholevelocityJacobian(LL,rr,R_B5,T{i},uj,vj,wj,ud,vd,wd);
%     veloomega=JTV*dottheta;veloomega_h(:,i)=veloomega;
end
figure;
plot(vomega_h(1:3,3:end).','LineWidth',2)
legend('d_x', 'd_y','d_z')
word=['第',num2str(j),'个关节角有0.01误差时末端执行器的移动微分相对误差'];
title(word)
saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\绝对误差\','关节',num2str(j),'移动.jpg']);
% figure
% plot(veloomega_h(1:3,3:end).','LineWidth',2)
% legend('v_x', 'v_y','v_z')
figure
plot(vomega_h(4:6,3:end).','LineWidth',2)
legend('\delta_x', '\delta_y','\delta_z')
word=['第',num2str(j),'个关节角有0.01误差时末端执行器的转动微分相对误差'];
title(word)
saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\绝对误差\','关节',num2str(j),'转动.jpg']);
% figure
% plot(veloomega_h(4:6,3:end).','LineWidth',2)
% legend('\omega_x', '\omega_y','\omega_z')
end
%% 在固定误差上形象化表示，除以最小值
for j=1:7
    %%
for i=3:length(T)
    [LL,rr,R_B5,uj,vj,wj,ud,vd,wd]  = wholegetL(T{i});
    J = wholeJacobian(LL,rr,R_B5,uj,vj,wj,ud,vd,wd);
    dottheta=zeros(7,1);
    dottheta(j)=0.01;
    spacevomega=J*dottheta;
    R_BT=T{i}(1:3,1:3);p=T{i}(1:3,4);pp=T{i}(1:4,4);
    ppdot=skewV(spacevomega)*pp;
    vomega=spacevomega;vomega(1:3)=ppdot(1:3);
    vomega_h(:,i)=vomega;
end
figure
temp=vomega_h(1:3,3:end);
ord=abs(temp)==min(abs(temp));
conjtemp=temp(ord);
vrds=temp.'./conjtemp;
plot(vrds,'LineWidth',2)
legend('d_x', 'd_y','d_z')
word=['第',num2str(j),'个关节角有误差时末端执行器的移动微分相对误差'];
title(word)
saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\除以最小值\','关节',num2str(j),'转动.jpg']);
figure
temp=vomega_h(4:6,3:end);
ord=abs(temp)==min(abs(temp));
conjtemp=temp(ord);
yids=temp.'./conjtemp;
plot(yids,'LineWidth',2)
legend('\delta_x', '\delta_y','\delta_z')
word=['第',num2str(j),'个关节角有误差时末端执行器的转动微分相对误差'];
title(word)
saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\除以最小值\','关节',num2str(j),'转动.jpg']);
end
%% 在固定误差上形象化表示，除以最大值
for j=1:7
    %%
for i=3:length(T)
    [LL,rr,R_B5,uj,vj,wj,ud,vd,wd]  = wholegetL(T{i});
    J = wholeJacobian(LL,rr,R_B5,uj,vj,wj,ud,vd,wd);
    dottheta=zeros(7,1);
    dottheta(j)=0.01;
    spacevomega=J*dottheta;
    R_BT=T{i}(1:3,1:3);p=T{i}(1:3,4);pp=T{i}(1:4,4);
    ppdot=skewV(spacevomega)*pp;
    vomega=spacevomega;vomega(1:3)=ppdot(1:3);
    vomega_h(:,i)=vomega;
end
figure
temp=vomega_h(1:3,3:end);
vrds=abs(temp)./max(abs(temp));
plot(vrds.','LineWidth',2)
legend('d_x', 'd_y','d_z')
word=['第',num2str(j),'个关节角有误差时末端执行器的移动微分相对误差'];
title(word)
saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\除以最大值\','关节',num2str(j),'移动.jpg']);
figure
temp=vomega_h(4:6,3:end);
yids=abs(temp)./max(abs(temp));
plot(yids.','LineWidth',2)
legend('\delta_x', '\delta_y','\delta_z')
word=['第',num2str(j),'个关节角有误差时末端执行器的转动微分相对误差'];
title(word)
saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\除以最大值\','关节',num2str(j),'转动.jpg']);
end
%% 保存7个关节的关节误差
for j=1:7
for i=3:length(T)
    [LL,rr,R_B5,uj,vj,wj,ud,vd,wd]  = wholegetL(T{i});
    J = wholeJacobian(LL,rr,R_B5,uj,vj,wj,ud,vd,wd);
    dottheta=zeros(7,1);
    dottheta(j)=0.01;
    spacevomega=J*dottheta;
    R_BT=T{i}(1:3,1:3);p=T{i}(1:3,4);pp=T{i}(1:4,4);
    ppdot=skewV(spacevomega)*pp;
    vomega=spacevomega;vomega(1:3)=ppdot(1:3);
    vomega_h(:,i)=vomega;
end
save(strcat('E:\大学文件\科研\毕业设计\20220322程序汇总\vomega_h',num2str(j)),'vomega_h');
end
%% 加载误差
load('vomega_h1.mat')
vomega_h1=vomega_h(:,3:end);
load('vomega_h2.mat')
vomega_h2=vomega_h(:,3:end);
load('vomega_h3.mat')
vomega_h3=vomega_h(:,3:end);
load('vomega_h4.mat')
vomega_h4=vomega_h(:,3:end);
load('vomega_h5.mat')
vomega_h5=vomega_h(:,3:end);
load('vomega_h6.mat')
vomega_h6=vomega_h(:,3:end);
load('vomega_h7.mat')
vomega_h7=vomega_h(:,3:end);
%% 加载循环
clear;
vomgea_h=cell(7,1);
for i=1:7
    wor=['vomega_h',num2str(i),'.mat'];
    load(wor)
    vomgea_h{i}=vomega_h(:,3:end);
end
%% 固定误差除以最大值 纵向对比
dx=zeros(length(vomgea_h{1}),7);
dy=zeros(length(vomgea_h{1}),7);
dz=zeros(length(vomgea_h{1}),7);
deltax=zeros(length(vomgea_h{1}),7);
deltay=zeros(length(vomgea_h{1}),7);
deltaz=zeros(length(vomgea_h{1}),7);
for i=1:length(vomgea_h{1})
    dx(i,:)=[vomgea_h{1}(1,i),vomgea_h{2}(1,i),vomgea_h{3}(1,i),vomgea_h{4}(1,i),vomgea_h{5}(1,i),vomgea_h{6}(1,i),vomgea_h{7}(1,i)];
    dy(i,:)=[vomgea_h{1}(2,i),vomgea_h{2}(2,i),vomgea_h{3}(2,i),vomgea_h{4}(2,i),vomgea_h{5}(2,i),vomgea_h{6}(2,i),vomgea_h{7}(2,i)];
    dz(i,:)=[vomgea_h{1}(3,i),vomgea_h{2}(3,i),vomgea_h{3}(3,i),vomgea_h{4}(3,i),vomgea_h{5}(3,i),vomgea_h{6}(3,i),vomgea_h{7}(3,i)];
    deltax(i,:)=[vomgea_h{1}(4,i),vomgea_h{2}(4,i),vomgea_h{3}(4,i),vomgea_h{4}(4,i),vomgea_h{5}(4,i),vomgea_h{6}(4,i),vomgea_h{7}(4,i)];
    deltay(i,:)=[vomgea_h{1}(5,i),vomgea_h{2}(5,i),vomgea_h{3}(5,i),vomgea_h{4}(5,i),vomgea_h{5}(5,i),vomgea_h{6}(5,i),vomgea_h{7}(5,i)];
    deltaz(i,:)=[vomgea_h{1}(6,i),vomgea_h{2}(6,i),vomgea_h{3}(6,i),vomgea_h{4}(6,i),vomgea_h{5}(6,i),vomgea_h{6}(6,i),vomgea_h{7}(6,i)];
end
%% 绝对误差
figure
plot(dx,'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
word=['dx'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比除以最大值\dx.jpg');
figure
plot(dy,'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
word=['dy'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比除以最大值\dy.jpg');
figure
plot(dz,'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
word=['dz'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比除以最大值\dz.jpg');
figure
plot(deltax,'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
word=['deltax'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比除以最大值\deltax.jpg');
figure
plot(deltay,'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
word=['deltay'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比除以最大值\deltay.jpg');
figure
plot(deltaz,'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
word=['deltaz'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比除以最大值\deltaz.jpg');

%% 相对误差，除以最大值100%
d=cell(6,1);
for i=1:length(vomgea_h{1})
    for j=1:6
    d{j}(i,:)=[vomgea_h{1}(j,i),vomgea_h{2}(j,i),vomgea_h{3}(j,i),vomgea_h{4}(j,i),vomgea_h{5}(j,i),vomgea_h{6}(j,i),vomgea_h{7}(j,i)];
    end
end
% save('E:\大学文件\科研\毕业设计\20220322程序汇总\joid','joid');

%% 形象化表示,除以最大值
word=["d_x";"d_y";"d_z";"\delta_x";"\delta_y";"\delta_z"];
for i=1:6
    figure
    temp=d{i}.';
    vrds=abs(temp)./max(abs(temp));
    plot(vrds.','LineWidth',2)
    legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
    title(word(i))
    saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比关节误差除以最大值\',num2str(i),'.jpg']);
end
%% 形象化表示,除以最小值
word=["d_x";"d_y";"d_z";"\delta_x";"\delta_y";"\delta_z"];
for i=1:6
    figure
    temp=d{i}.';
    vrds=abs(temp)./min(abs(temp));
    plot(vrds.','LineWidth',2)
    legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
    title(word(i))
%     saveas(gcf,['E:\大学文件\科研\毕业设计\20220415进度汇报\纵向对比关节误差除以最小值\',num2str(i),'.jpg']);
end
%% 辐射图test
joint=[0 1 2 3 4 5 6 7]*pi*2/7;
rho=[1 2 3 4 5 4 3 1];
polarplot(joint,rho);
        thetaticks(0:360/7:360/7*6)
  rlim([-5 15])
%% 辐射各个微分图
word=["d_x";"d_y";"d_z";"\delta_x";"\delta_y";"\delta_z"];
str=["关节1","关节2","关节3","关节4","关节5","关节6","关节7"];
joint=[0 1 2 3 4 5 6 7]*pi*2/7;
for i=1:6
    h=figure;
    set(h,'position',[100 100 1000 1000]);
    temp=abs(d{i}.');
    temp=[temp;temp(1,:)];
        polarplot(joint,temp,'g-','LineWidth',2,'marker','.','markeredgecolor','b','markersize',20);
        hold on
        thetaticks(0:360/7:360/7*6)
        thetaticklabels(str)
%         for k=1:7
%             str=['关节',num2str(k)];
%             text(joint(k),temp(k,i),str);
%         end
        if i<3.5
            rlim([0 5])
        else
            rlim([0 0.01])
        end
    title(word(i))
%     saveas(gcf,['jointerror',num2str(i),'.jpg']);
end
%% 辐射各个微分动图
for i=5:6
    %%
    word=["d_x";"d_y";"d_z";"\delta_x";"\delta_y";"\delta_z"];
    joint=[0 1 2 3 4 5 6 7]*pi*2/7;
    figure(i)
    pic_num = 1;
    temp=abs(d{i}.');
    temp=[temp;temp(1,:)];
    for j=1:length(temp)
        polarplot(joint,temp(:,j),'LineWidth',2,'marker','.','markeredgecolor','b','markersize',20);
        for k=1:7
            str=['关节',num2str(k)];
            text(joint(k),temp(k,j),str);
        end      
        thetaticks(0:360/7:360/7*6)
        if i<3.5
            rlim([0 5])
        else
            rlim([0 0.01])
        end
        drawnow; 
        title(word(i))
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        name=[num2str(i),'.gif'];
        if pic_num == 1
            imwrite(I,map,name,'gif', 'Loopcount',inf,'DelayTime',0.1);
        else
            imwrite(I,map,name,'gif','WriteMode','append','DelayTime',0.1);
        end
        pic_num = pic_num + 1;       
    end
end
%% 画在一张大图上，动图
    %% i=1~3
    word=["d_x";"d_y";"d_z";"\delta_x";"\delta_y";"\delta_z"];
    joint=[0 1 2 3 4 5 6 7]*pi*2/7;
    figure(1)
    pic_num = 1;
    temp1=abs(d{1}.');
    temp1=[temp1;temp1(1,:)];
    temp2=abs(d{2}.');
    temp2=[temp2;temp2(1,:)];
    temp3=abs(d{3}.');
    temp3=[temp3;temp3(1,:)];
    for j=1:length(temp1)
        temp=[temp1(:,j),temp2(:,j),temp3(:,j)];
        polarplot(joint,temp,'LineWidth',2,'marker','.','markeredgecolor','b','markersize',20);
%         for k=1:7            
%             str=['关节',num2str(k)];
%             text(joint(k),temp(k,1),str);
%         end      
%         if i<3.5
    rlim([0 5])
%         else
%             rlim([0 0.01])
%         end
        title('移动微分')
        thetaticks(0:360/7:360/7*6)
        legend('d_x','d_y','d_z');
        drawnow; 
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        name='移动微分关节7个.gif';
        if pic_num == 1
            imwrite(I,map,name,'gif', 'Loopcount',inf,'DelayTime',0.1);
        else
            imwrite(I,map,name,'gif','WriteMode','append','DelayTime',0.1);
        end
        pic_num = pic_num + 1;       
    end
        %% i=4~6
    word=["d_x";"d_y";"d_z";"\delta_x";"\delta_y";"\delta_z"];
    joint=[0 1 2 3 4 5 6 7]*pi*2/7;
    figure(1)
    pic_num = 1;
    temp1=abs(d{4}.');
    temp1=[temp1;temp1(1,:)];
    temp2=abs(d{5}.');
    temp2=[temp2;temp2(1,:)];
    temp3=abs(d{6}.');
    temp3=[temp3;temp3(1,:)];
    for j=1:length(temp1)
        temp=[temp1(:,j),temp2(:,j),temp3(:,j)];
        polarplot(joint,temp,'LineWidth',2,'marker','.','markeredgecolor','b','markersize',20);
%         for k=1:7            
%             str=['关节',num2str(k)];
%             text(joint(k),temp(k,1),str);
%         end      
%         if i<3.5
%     rlim([0 5])
%         else
            rlim([0 0.01])
%         end
        thetaticks(0:360/7:360/7*6)
        title('转动微分')
        legend('\omega_x','\omega_y','\omega_z');
        drawnow; 
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        name='转动微分关节7个.gif';
        if pic_num == 1
            imwrite(I,map,name,'gif', 'Loopcount',inf,'DelayTime',0.1);
        else
            imwrite(I,map,name,'gif','WriteMode','append','DelayTime',0.1);
        end
        pic_num = pic_num + 1;       
    end