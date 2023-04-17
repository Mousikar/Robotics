%% 整体机械臂正解测试
clear
a=linspace(0,pi/9,100);
b=linspace(0,pi/6,100);
c=linspace(0,pi/11,100);
r=linspace(0,pi/7,100);
a1=linspace(0,pi/9,100);
b1=linspace(0,pi/7,100);
c1=linspace(0,pi/10,100);
temp=diag([1 1 1 1]);
temp(:,4)=[0 0 538 0 ].';
T_history=cell(100,1);
T_history{1}=temp;
figure;
for i=1:100
    q=[a(i) b(i) c(i) r(i) a1(i) b1(i) c1(i)];
    T_BT = wholeforwardPro(q);
%     tranimate(temp,T_BT);
    temp= T_BT;
    plot3(temp(1,4),temp(2,4),temp(3,4),'ro','MarkerFaceColor','r')
    hold on;
%     axis([0 538 0 538 0 538])
    T_history{i}=T_BT;
    pause(0.1)
end
%% 
figure   
for i=2:100
    t=T_history{i};     
    plot3(t(1,4),t(2,4),t(3,4),'ro','MarkerFaceColor','r')
    hold on;
end 
%% 整体机械臂反解测试
count=1;
for i=3:100
    T_BT=T_history{i};
%     axis([0 300 0 400 0 600])
    q = wholeinversePro(T_BT);
    q_hitory(:,count)=q(:,1);
    count=count+1;
    
    T_BT_history{i} = wholeforward(q);
%     
%     [M,p,u,v,w] = RRR3expprod(q(1),q(2),q(3));
%     [theta,psi] = RRR3inverse(M);  
%     
    pause(0.2);
%     clf;
end

% 检查反解是否正确
for i=3:length(T_history)    
    if norm(T_history{i}-T_BT_history{i})>=0.0001
        disp('wrong')
    else
        disp('right')
    end
end
disp('check over')
%% 反解测试画圆
% for i=1:10:length(T)
count=1;
r=70;
for theta=0:pi/20:2*pi
    T_BT=T_history{50};
    T_BT(1,4)=T_BT(1,4)-50+r*cos(theta);
    T_BT(2,4)=T_BT(2,4)-50+r*sin(theta);
%     axis([0 300 0 400 0 600])
    q = wholeinversePro(T_BT);
    q_hitory(:,count)=q(:,1);
    count=count+1;
%     
%     [M,p,u,v,w] = RRR3expprod(q(1),q(2),q(3));
%     [theta,psi] = RRR3inverse(M);  
%     
%     pause(0.1);
%     clf;
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
%% 写字
count=1;
for i=1:20:length(T)
    T_BT=T{i};
%     axis([0 300 0 400 0 600])
    q = wholeinversePro(T_BT);
    q_hitory(:,count)=q(:,1);
    count=count+1;
%     pause(0.01);
%     clf;
end
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
%% 整体机械臂指数积公式正解测试
a=linspace(0,pi/9,100);
b=linspace(0,pi/6,100);
c=linspace(0,pi/11,100);
r=linspace(0,pi/9,100);
a1=linspace(0,pi/9,100);
b1=linspace(0,pi/6,100);
c1=linspace(0,pi/11,100);
temp=diag([1 1 1 1]);
temp(:,4)=[0 0 538 0 ].';
T_POEhistory=cell(100,1);
T_POEhistory{1}=temp;
for i=2:100
    q=[a(i) b(i) c(i) r(i) a1(i) b1(i) c1(i)];
    T_BT = wholePOEforward(q);
    axis([0 538 0 538 0 538])
%     tranimate(temp,T_BT);
    hold on;
    plot3(temp(1,4),temp(2,4),temp(3,4),'bo','MarkerFaceColor','b')
    temp= T_BT;
    T_POEhistory{i}=temp;
end
%% 对比两种正解方法
for i=1:length(T_history)    
    if norm(T_history{i}-T_POEhistory{i})>=0.0001
        disp('wrong')
    else
        disp('right')
    end
end
disp('check over')
%% 整体机械臂指数积反解测试
T_BT_history=cell(100,1);
count=1;
qtem=0;
for i=3:100
    T_BT=T_history{i};
%     clf;
    qq = wholePOEinversePro(T_BT);
    siz=qq-qtem;
    q=qq(:,sum(siz.*siz,1)==min(sum(siz.*siz,1)));
    qtem=q;
    q_hitory(:,count)=q(:,1);
    count=count+1;
    T_BT_history{i} = wholeforwardPro(q);
%     T_BT_history{i} = wholePOEforward(q);
end
% 检查反解是否正确
for i=3:length(T_history)    
    if norm(T_history{i}-T_BT_history{i})>=0.0001
        disp('wrong')
    else
        disp('right')
    end
end
disp('check over')
%% 肩关节和腕关节动作
figure;
for i=1:length(q_hitory) 
    [M,p,u,v,w] = RRR3expprod(q_hitory(1,i),q_hitory(2,i),q_hitory(3,i));
    [theta,psi] = RRR3inversePro(M);    
    pause(0.2);
    clf;
end
figure;
for i=1:length(q_hitory) 
    [M,p,u,v,w] = RRR3expprod(q_hitory(5,i),q_hitory(6,i),q_hitory(7,i));
    [theta,psi] = RRR3inversePro(M);    
    pause(0.2);
    clf;
end
%% 画圆
load('T_history.mat')
r=70;
count=1;
for theta=0:pi/10:2*pi
    T_BT=T_history{50};
    T_BT(1,4)=T_BT(1,4)-50+r*cos(theta);
    T_BT(2,4)=T_BT(2,4)-50+r*sin(theta);
    q = wholePOEinverse(T_BT);
    q_hitory(:,count)=q(:,1);
    count=count+1;
    pause(0.1);
end
%% 写字
count=1;
T_BT_history=cell(length(T),1);
for i=1:20:length(T)
    T_BT=T{i};
%     axis([0 300 0 400 0 600])
    q = wholePOEinverse(T_BT);
    T_BT_history{i}=wholeforward(q);
    q_hitory(:,count)=q(:,1);
    count=count+1;
    pause(0.01);
%     clf;
end
% 检查
for i=1:20:length(T)    
    if norm(T{i}-T_BT_history{i})>=0.0001
        disp('wrong')
    else
        disp('right')
    end
end
disp('check over')
%% 整体机械臂随机给关节角，得到工作空间
num=10000;
T_POEhistory=cell(num,1);
T_POEhistory{1}=temp;
for i=1:num
    q=zeros(7,1);
    q(1:3)=-pi/6+2*pi/3*rand([3,1]);
%     q(4)=5*pi/6*rand(1);    
    q(4)=-pi+pi*rand(1); 
    q(5:7)=-pi/6+2*pi/3*rand([3,1]);
    T_BT = wholePOEforward(q);    
    T_POEhistory{i}=T_BT;
end
%% 画图
x=zeros(num,1);
y=zeros(num,1);
z=zeros(num,1);
for i=1:num
    T=T_POEhistory{i};
    x(i)=T(1,4);
    y(i)=T(2,4);
    z(i)=T(3,4);
    plot3(T(1,4),T(2,4),T(3,4),'b.','MarkerFaceColor','b')
    hold on;grid on;
end
% % 凸包
% figure
% [K,V] = convhull(x,y,z);
% trisurf(K,x,y,z,'Facecolor','cyan')
%%
figure
plot(sqrt(x.^2+y.^2+z.^2))
max(ans)