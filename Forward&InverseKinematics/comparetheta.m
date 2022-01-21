function  next_theta = comparetheta( a_theta,target )
%comparetheta 求解最近的关节角
% 输入：当前位置所对应的关节角，目标姿态
% 离现在关节角最近的关节角
t_theta=InverseTransformation(target);%t_theta为元胞数组
for i=1:8
%         if t_theta{i}(1)<a_theta(1)*1.1 && t_theta{i}(1)>a_theta(1)*0.9 && t_theta{i}(2)<a_theta(2)*1.1 && t_theta{i}(2)>a_theta(2)*0.9 && ...
%             t_theta{i}(3)<a_theta(3)*1.1 && t_theta{i}(3)>a_theta(3)*0.9 && t_theta{i}(4)<a_theta(4)*1.1 && t_theta{i}(4)>a_theta(4)*0.9 && ...
%             t_theta{i}(5)<a_theta(5)*1.1 && t_theta{i}(5)>a_theta(5)*0.9 && t_theta{i}(6)<a_theta(6)*1.1 && t_theta{i}(6)>a_theta(6)*0.9
        if abs(t_theta{i}-a_theta)<=3*a_theta
            flag=i;
            disp(i);
            disp(a_theta);
        end
end
next_theta=t_theta{flag};
end