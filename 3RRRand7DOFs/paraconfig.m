function [gamma,beta,alpha_1,alpha_2,eta,a_3,a_4,d_7] = paraconfig(err)
%paraconfig 设定结构参数
    if nargin==0
        err=[0 0 0 0 0 0 0 0 0 0 0 0].';
    end
%% 角度
gamma=60/180*pi*ones(3,1)+err(1:3);
beta=45/180*pi*ones(3,1)+err(7:9);
alpha_1=pi/2*ones(3,1)+err(4:6);
alpha_2=pi/2*ones(3,1);
eta=[0 2/3*pi 4/3*pi].'+err(10:12);
%% 长度
a_3=255;
a_4=215;
d_7=68;
end

