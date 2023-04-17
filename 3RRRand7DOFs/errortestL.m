% errortestL.m
%% 一组位姿测试
clear
phi=[pi/11,pi/12,pi/13];
T=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
[theta,phi,xi] = RRR3expprodinvPro(T);
[gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig();

JL = JacobL(gamma,alpha_1,beta,eta,phi,theta)
delta_L=[0.01 0 0 0 0 0].';
delta_x=JL*delta_L
R_dphi2domega = dphi2domega(phi)
delta=R_dphi2domega*delta_x
%% 一组位姿的误差
clear
num=100;
a=linspace(0,pi/11,num);
b=linspace(0,pi/9,num);
c=linspace(0,pi/10,num);
for i=1:num
    phi=[a(i),b(i),c(i)];
    T=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
    [theta,phi,xi] = RRR3expprodinvPro(T);
    [gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig();

    JL = JacobL(gamma,alpha_1,beta,eta,phi,theta);
    delta_L=[ 0.01 0 0 0 0 0 ].';
    delta_x=JL*delta_L;
    R_dphi2domega = dphi2domega(phi);
    delta=R_dphi2domega*delta_x;
    dotomega(:,i)=delta;
end
% plot(dotomega.')
plot1 = plot(dotomega.','LineWidth',2);
legend('\omega_x', '\omega_y','\omega_z')
%% 改进JacobL测试
clear
phi=[pi/11,pi/12,pi/13];
T=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
[theta,~,xi] = RRR3expprodinvPro(T);
[gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig();

JL = JacobLPro(gamma,alpha_1,beta,eta,phi,theta)
delta_L=[0.01 0 0 0 0 0 0 0 0 0 0 0].';
delta_x=JL*delta_L
R_dphi2domega = dphi2domega(phi)
J=R_dphi2domega*JL
delta=R_dphi2domega*delta_x
%% 改进一组测试
clear
num=100;
a=linspace(0,pi/11,num);
b=linspace(0,pi/9,num);
c=linspace(0,pi/10,num);
wor=["\gamma_1" "\gamma_2" "\gamma_3" "\alpha_{11}" "\alpha_{12}" "\alpha_{13}" "\beta_1" "\beta_2" "\beta_3" "\eta_1" "\eta_2" "\eta_3"];
for j=1:12
for i=1:num
    phi=[a(i),b(i),c(i)];
    T=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
    [theta,~,xi] = RRR3expprodinvPro(T);
    [gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig();

    JL = JacobLPro(gamma,alpha_1,beta,eta,phi,theta);
    delta_L=zeros(12,1);
    delta_L(j)=0.01;
    delta_x=JL*delta_L;
    R_dphi2domega = dphi2domega(phi);
    delta=R_dphi2domega*delta_x;
    dotomega(:,i)=delta;
end
figure
plot(dotomega.','LineWidth',2);
title(wor(j))
legend('\omega_x', '\omega_y','\omega_z')
saveas(gcf,['E:\大学文件\科研\毕业设计\20220429进度汇报\结构参数误差\',num2str(j),'.jpg']);

end
%% 校准模型
% 有误差
clear;clc
err=[ 0 0 0 0 0 0 0.02 0 0.05 0 0 0].';%真实误差
err_sol=zeros(12,1);%认为误差
sum=err_sol;%累计误差
q=[pi/6 pi/10 pi/9].';%电机输入
[R_err,~,~,~,~] = RRR3expprodPro(q,err);
[a, b, c]=r2zyz(R_err);phi_err=[a b c].';%真实输出x‘
[R,~,~,~,~] = RRR3expprodPro(q,sum);
[a, b, c]=r2zyz(R);phi=[a b c].';%计算结果输出x
%% 
global JL
global delta_phi
delta_phi=phi_err-phi%残差
opts=optimoptions(@fsolve);
opts.Algorithm="levenberg-marquardt";
[gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig(sum);
JL = JacobLPro(gamma,alpha_1,beta,eta,phi,q);
err_sol=fsolve(@F,zeros(12,1),opts);%
a=F(err_sol);
sum=sum+err_sol;
[R,~,~,~,~] = RRR3expprodPro(q,sum);
[a, b, c]=r2zyz(R);phi=[a b c].';%计算结果输出x
delta_phi=phi_err-phi;%残差
delta_phi
%% 换一种方式，将误差都等效在alpha上
global JL
global delta_phi
delta_phi=phi_err-phi%残差
opts=optimoptions(@fsolve);
opts.Algorithm="levenberg-marquardt";
[gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig(sum);
JL = JacobLPro(gamma,alpha_1,beta,eta,phi,q);
alpha_err=fsolve(@F,zeros(3,1),opts);%
    err_sol=zeros(12,1);
    err_sol(4:6)=alpha_err;
sum=sum+err_sol;
[R,~,~,~,~] = RRR3expprodPro(q,sum);
[a, b, c]=r2zyz(R);phi=[a b c].';%计算结果输出x
delta_phi=phi_err-phi;%残差
delta_phi
%% 整体机械臂结构误差测试
clear
num=100;
load('T_history.mat');
T=T_history;
%%
wor=["定平台\gamma_1" "定平台\gamma_2" "定平台\gamma_3" "定平台\alpha_{11}" "定平台\alpha_{12}" "定平台\alpha_{13}" "定平台\beta_1" "定平台\beta_2" "定平台\beta_3" "定平台\eta_1" "定平台\eta_2" "定平台\eta_3"...
    "\alpha_3" "a_3" "d_4" "\alpha_4" "a_4" "d_5"...
    "动平台\gamma_1" "动平台\gamma_2" "动平台\gamma_3" "动平台\alpha_{11}" "动平台\alpha_{12}" "动平台\alpha_{13}" "动平台\beta_1" "动平台\beta_2" "动平台\beta_3" "动平台\eta_1" "动平台\eta_2" "动平台\eta_3"
];
for j=1:30
    %%
for i=3:num
    %%
    [LL,rr,R_B5,qq,phi_j,phi_d,x_4,x_3_] = wholeL_getL(T{i});
    J = wholeL_Jacobian(LL,rr,R_B5,qq,phi_j,phi_d,x_4,x_3_);
    
    dotl=zeros(30,1);
    dotl(j)=0.01;
    spacevomega=J*dotl;
    R_BT=T{i}(1:3,1:3);p=T{i}(1:3,4);pp=T{i}(1:4,4);
    ppdot=skewV(spacevomega)*pp;
    vomega=spacevomega;vomega(1:3)=ppdot(1:3);
    vomega_h(:,i)=vomega;
end
save(strcat('E:\大学文件\科研\毕业设计\20220322程序汇总\Lvomega_h',num2str(j)),'vomega_h');
%%
figure;
plot(vomega_h(1:3,3:end).','LineWidth',2)
legend('d_x', 'd_y','d_z')
% word=['第',num2str(j),'个结构参数有0.01误差时末端执行器的移动微分相对误差'];
title(wor(j))
saveas(gcf,['E:\大学文件\科研\毕业设计\20220429进度汇报\整体机械臂结构参数误差\','结构参数',num2str(j),'移动.jpg']);
figure
plot(vomega_h(4:6,3:end).','LineWidth',2)
legend('\delta_x', '\delta_y','\delta_z')
% word=['第',num2str(j),'个结构参数有0.01误差时末端执行器的转动微分相对误差'];
title(wor(j))
saveas(gcf,['E:\大学文件\科研\毕业设计\20220429进度汇报\整体机械臂结构参数误差\','结构参数',num2str(j),'转动.jpg']);
end
%% 加载循环
clear;
Lvomgea_hall=cell(30,1);
for i=1:30
    wor=['Lvomega_h',num2str(i),'.mat'];
    load(wor)
    Lvomgea_hall{i}=vomega_h(:,3:end);
end
save(strcat('E:\大学文件\科研\毕业设计\20220322程序汇总\Lvomega_hall'),'Lvomgea_hall');
%% 固定误差除以最大值 纵向对比
dx=zeros(length(Lvomgea_hall{1}),30);
dy=zeros(length(Lvomgea_hall{1}),30);
dz=zeros(length(Lvomgea_hall{1}),30);
deltax=zeros(length(Lvomgea_hall{1}),30);
deltay=zeros(length(Lvomgea_hall{1}),30);
deltaz=zeros(length(Lvomgea_hall{1}),30);
for i=1:length(Lvomgea_hall{1})
    for j=1:30
        dx(i,j)=Lvomgea_hall{j}(1,i);
        dy(i,j)=Lvomgea_hall{j}(2,i);
        dz(i,j)=Lvomgea_hall{j}(3,i);
        deltax(i,j)=Lvomgea_hall{j}(4,i);
        deltay(i,j)=Lvomgea_hall{j}(5,i);
        deltaz(i,j)=Lvomgea_hall{j}(6,i);
    end
end
%%
wor=["定平台\gamma_1" "定平台\gamma_2" "定平台\gamma_3" "定平台\alpha_{11}" "定平台\alpha_{12}" "定平台\alpha_{13}" "定平台\beta_1" "定平台\beta_2" "定平台\beta_3" "定平台\eta_1" "定平台\eta_2" "定平台\eta_3"...
    "\alpha_3" "a_3" "d_4" "\alpha_4" "a_4" "d_5"...
    "动平台\gamma_1" "动平台\gamma_2" "动平台\gamma_3" "动平台\alpha_{11}" "动平台\alpha_{12}" "动平台\alpha_{13}" "动平台\beta_1" "动平台\beta_2" "动平台\beta_3" "动平台\eta_1" "动平台\eta_2" "动平台\eta_3"
];
figure
plot(dx,'LineWidth',2)
% legend1 = legend(wor);
% set(legend1,...
%     'Position',[0.152213541666667 0.716694078947368 0.4796875 0.185307017543859],...
%     'NumColumns',6);
word=['dx'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220429进度汇报\纵向对比绝对误差\dx.jpg');
figure
plot(dy,'LineWidth',2)
% legend1 = legend(wor);
% set(legend1,...
%     'Position',[0.152213541666667 0.716694078947368 0.4796875 0.185307017543859],...
%     'NumColumns',6);
word=['dy'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220429进度汇报\纵向对比绝对误差\dy.jpg');
figure
plot(dz,'LineWidth',2)
% legend1 = legend(wor);
% set(legend1,...
%     'Position',[0.152213541666667 0.716694078947368 0.4796875 0.185307017543859],...
%     'NumColumns',6);
word=['dz'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220429进度汇报\纵向对比绝对误差\dz.jpg');
figure
plot(deltax,'LineWidth',2)
% legend1 = legend(wor);
% set(legend1,...
%     'Position',[0.152213541666667 0.716694078947368 0.4796875 0.185307017543859],...
%     'NumColumns',6);
% word=['\delta_x'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220429进度汇报\纵向对比绝对误差\deltax.jpg');
figure
plot(deltay,'LineWidth',2)
% legend1 = legend(wor);
% set(legend1,...
%     'Position',[0.152213541666667 0.716694078947368 0.4796875 0.185307017543859],...
%     'NumColumns',6);
% word=['\delta_y'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220429进度汇报\纵向对比绝对误差\deltay.jpg');
figure
plot(deltaz,'LineWidth',2)
% legend1 = legend(wor);
% set(legend1,...
%     'Position',[0.152213541666667 0.716694078947368 0.4796875 0.185307017543859],...
%     'NumColumns',6);
% word=['\delta_z'];
title(word)
saveas(gcf,'E:\大学文件\科研\毕业设计\20220429进度汇报\纵向对比绝对误差\deltaz.jpg');
%% 相对误差，除以最大值100%
pard=cell(6,1);
for i=1:length(Lvomgea_hall{1})
    for j=1:6
        for k=1:30
            pard{j}(i,k)=Lvomgea_hall{k}(j,i);
        end
    end
end
save('E:\大学文件\科研\毕业设计\20220322程序汇总\pard','pard');
%%
            pard=load('pard.mat');
            plotdata=cell(6,1);
            for i=1:6
                temp=pard.pard{i}.';
                count=1;
                for j=1:30
                    if temp(j,:)~=0
                    tem(count,:)=temp(j,:);
                    count=count+1;
                    end
                end
                vrds=abs(tem)./min(abs(tem));
                plotdata{i}=vrds.';
            end
            figure
            plot(plotdata{1}(10:end,:),'LineWidth',2)
            figure
            plot(plotdata{2}(10:end,:),'LineWidth',2)
            figure
            plot(plotdata{3}(10:end,:),'LineWidth',2)
            figure
            plot(plotdata{4}(10:end,:),'LineWidth',2)
            figure
            plot(plotdata{5}(10:end,:),'LineWidth',2)
            figure
            plot(plotdata{6}(10:end,:),'LineWidth',2)
%% 辐射各个微分图
word=["d_x";"d_y";"d_z";"\delta_x";"\delta_y";"\delta_z"];
for i=1:31
    joi(i)=i-1;
end
joint=joi*pi*2/30;            
% str=["定平台\gamma_1" "定平台\gamma_2" "定平台\gamma_3" "定平台\alpha_{11}" "定平台\alpha_{12}" "定平台\alpha_{13}" "定平台\beta_1" "定平台\beta_2" "定平台\beta_3" "定平台\eta_1" "定平台\eta_2" "定平台\eta_3"...
%     "\alpha_3" "a_3" "d_4" "\alpha_4" "a_4" "d_5"...
%     "动平台\gamma_1" "动平台\gamma_2" "动平台\gamma_3" "动平台\alpha_{11}" "动平台\alpha_{12}" "动平台\alpha_{13}" "动平台\beta_1" "动平台\beta_2" "动平台\beta_3" "动平台\eta_1" "动平台\eta_2" "动平台\eta_3"
% ];
str=["j_\gamma_1" "j_\gamma_2" "j_\gamma_3" "j_\alpha_{11}" "j_\alpha_{12}" "j_\alpha_{13}" "j_\beta_1" "j_\beta_2" "j_\beta_3" "j_\eta_1" "j_\eta_2" "j_\eta_3"...
    "\alpha_3" "a_3" "d_4" "\alpha_4" "a_4" "d_5"...
    "w_\gamma_1" "w_\gamma_2" "w_\gamma_3" "w_\alpha_{11}" "w_\alpha_{12}" "w_\alpha_{13}" "w_\beta_1" "w_\beta_2" "w_\beta_3" "w_\eta_1" "w_\eta_2" "w_\eta_3"
];
%%

for i=1:6
    %%
    figure
    temp=abs(d{i}.');
    temp=[temp;temp(1,:)];
        polarplot(joint,temp.','g-','LineWidth',2,'marker','.','markeredgecolor','b','markersize',20);
        hold on
        thetaticks(0:360/30:360/30*29)
        thetaticklabels(str)
%         for k=1:30
%             text(joint(k),temp(k,end),str(k));
%         end
%         if i<3.5
%             rlim([0 5])
%         else
%             rlim([0 0.01])
%         end
    title(word(i))
%     saveas(gcf,['E:\大学文件\科研\毕业设计\20220429进度汇报\各个微分辐射图\',num2str(i),'.jpg']);
end
%% 函数区
function a=F(alpha_err)
    global JL
    global delta_phi
    err_sol=zeros(12,1);
    err_sol(4:6)=alpha_err;
    a=abs(JL*err_sol)-abs(delta_phi);
%     a=JL*err_sol-delta_phi;
end