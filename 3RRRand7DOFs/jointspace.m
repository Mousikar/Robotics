%% 肩关节关节空间
clear;
num=25;
qqhs=linspace(-90/180*pi,20/180*pi,num);
nswz=linspace(-90/180*pi,10/180*pi,num);
wxnx=linspace(-50/180*pi,40/180*pi,num);
% Q_history=cell(num^3,1);
% theta_history=zeros(3,8*num^3);
count=1;
hold on;axis equal;axis([-1 1 -1 1 -1 1]);grid on;xlabel('X');ylabel('Y');zlabel('Z')
for i=1:num
    for j=1:num
        for k=1:num
            cou=0;
            Qtemp=rotx(qqhs(i))*roty(nswz(j))*rotz(wxnx(k));
%             Q_history{count}=Qtemp;
            [theta,phi,xi] = RRR3expprodinvPro(Qtemp);

    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/3&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/3&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/3
        theta_history(:,count)=[theta(1,1) theta(2,1) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/3&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/3&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/3
        theta_history(:,count)=[theta(1,1) theta(2,1) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/3&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/3&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/3
        theta_history(:,count)=[theta(1,1) theta(2,2) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/3&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/3&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/3
        theta_history(:,count)=[theta(1,1) theta(2,2) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/3&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/3&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/3
        theta_history(:,count)=[theta(1,2) theta(2,1) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/3&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/3&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/3
        theta_history(:,count)=[theta(1,2) theta(2,1) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/3&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/3&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/3
        theta_history(:,count)=[theta(1,2) theta(2,2) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/3&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/3&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/3
        theta_history(:,count)=[theta(1,2) theta(2,2) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if cou>1
        disp(cou)
    end
%             if count>=2
%                 tranimate(Q_history{count-1},Q_history{count});
%             end
            plot3(Qtemp(1,3),Qtemp(2,3),Qtemp(3,3),'b.','MarkerFaceColor','b')
        end
    end
end
% rho=pi/5;
% theta_1=-pi:rho:pi;
% theta_2=-pi:rho:pi;
% theta_3=-pi:rho:pi;
%% 关节空间画出来
figure;
hold on;axis equal;
% axis(pi*[-1 1 -1 1 -1 1]);
grid on;xlabel('\theta_1');ylabel('\theta_2');zlabel('\theta_3')
count=1;
for i=1:length(theta_history)
%     if abs(theta_history(1,i))<=pi/1.5&&abs(theta_history(2,i))<=pi/1.5&&abs(theta_history(3,i))<=pi/1.5
        plot3(theta_history(1,i),theta_history(2,i),theta_history(3,i),'b.','MarkerFaceColor','b')
%         count=count+1;%count=1921
        hold on
%     end
end
%%
figure;
hold on;axis equal;
axis([-1 1 -1 1 -1 1]);
grid on;xlabel('x');ylabel('y');zlabel('z')
% global theta;
for i=1:length(theta_history)
    theta=[theta_history(1,i),theta_history(2,i),theta_history(3,i)].';
        [R,~] = RRR3expprodPro(theta);
        plot3(R(1,3),R(2,3),R(3,3),'b.','MarkerFaceColor','b')     
        hold on
end
%% 腕关节关节空间
clear;
num=25;
qqhs=linspace(-20/180*pi,45/180*pi,num);
nswz=linspace(-75/180*pi,75/180*pi,num);
wxnx=linspace(-76/180*pi,85/180*pi,num);
% qqhs=linspace(-20/180*pi,20/180*pi,num);
% nswz=linspace(-40/180*pi,40/180*pi,num);
% wxnx=linspace(-55/180*pi,15/180*pi,num);
% Q_history=cell(num^3,1);
% theta_history=zeros(3,8*num^3);
count=1;
hold on;axis equal;axis([-1 1 -1 1 -1 1]);grid on;xlabel('X');ylabel('Y');zlabel('Z')
for i=1:num
    for j=1:num
        for k=1:num
            cou=0;
            Qtemp=rotx(qqhs(i))*roty(nswz(j))*rotz(wxnx(k));
%             Q_history{count}=Qtemp;
            [theta,phi,xi] = RRR3expprodinvPro(Qtemp);

    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/2&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/2&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/2
        theta_history(:,count)=[theta(1,1) theta(2,1) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/2&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/2&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/2
        theta_history(:,count)=[theta(1,1) theta(2,1) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/2&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/2&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/2
        theta_history(:,count)=[theta(1,1) theta(2,2) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,1)>=-pi/6&&theta(1,1)<=pi/2&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/2&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/2
        theta_history(:,count)=[theta(1,1) theta(2,2) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/2&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/2&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/2
        theta_history(:,count)=[theta(1,2) theta(2,1) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/2&&...
            theta(2,1)>=-pi/6&&theta(2,1)<=pi/2&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/2
        theta_history(:,count)=[theta(1,2) theta(2,1) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/2&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/2&&...
            theta(3,1)>=-pi/6&&theta(3,1)<=pi/2
        theta_history(:,count)=[theta(1,2) theta(2,2) theta(3,1)].';
        count=count+1;cou=cou+1;
    end
    if theta(1,2)>=-pi/6&&theta(1,2)<=pi/2&&...
            theta(2,2)>=-pi/6&&theta(2,2)<=pi/2&&...
            theta(3,2)>=-pi/6&&theta(3,2)<=pi/2
        theta_history(:,count)=[theta(1,2) theta(2,2) theta(3,2)].';
        count=count+1;cou=cou+1;
    end
    if cou>1
        disp(cou)
    end
%             if count>=2
%                 tranimate(Q_history{count-1},Q_history{count});
%             end
            plot3(Qtemp(1,3),Qtemp(2,3),Qtemp(3,3),'b.','MarkerFaceColor','b')
        end
    end
end
% rho=pi/5;
% theta_1=-pi:rho:pi;
% theta_2=-pi:rho:pi;
% theta_3=-pi:rho:pi;
%% 关节空间画出来
figure;
hold on;axis equal;
% axis(pi*[-1 1 -1 1 -1 1]);
grid on;xlabel('\theta_1');ylabel('\theta_2');zlabel('\theta_3')
count=1;
for i=1:length(theta_history)
%     if abs(theta_history(1,i))<=pi/1.5&&abs(theta_history(2,i))<=pi/1.5&&abs(theta_history(3,i))<=pi/1.5
        plot3(theta_history(1,i),theta_history(2,i),theta_history(3,i),'b.','MarkerFaceColor','b')
%         count=count+1;%count=1921
        hold on
%     end
end
%%
figure;
hold on;axis equal;
axis([-1 1 -1 1 -1 1]);
grid on;xlabel('x');ylabel('y');zlabel('z')
% global theta;
for i=1:length(theta_history)
    theta=[theta_history(1,i),theta_history(2,i),theta_history(3,i)].';
        [R,~] = RRR3expprodPro(theta);
        plot3(R(1,3),R(2,3),R(3,3),'b.','MarkerFaceColor','b')     
        hold on
end
