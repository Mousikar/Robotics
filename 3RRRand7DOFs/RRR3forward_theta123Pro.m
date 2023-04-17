function [R,phi] = RRR3forward_theta123Pro(theta_1,theta_2,theta_3)
% 3RRR并联机器人正解
global theta_1
global theta_2
global theta_3
phi=zeros(3,8);
R=cell(8,1);
phi(:,1)=fsolve(@RRR3theta3myfun,[pi/2 pi/2 pi/2].');R{1}=rotz(phi(1,1))*roty(phi(2,1))*rotz(phi(3,1));
phi(:,2)=fsolve(@RRR3theta3myfun,[pi/2 pi/2 -pi/2].');R{2}=rotz(phi(1,2))*roty(phi(2,2))*rotz(phi(3,2));
phi(:,3)=fsolve(@RRR3theta3myfun,[pi/2 -pi/2 pi/2].');R{3}=rotz(phi(1,3))*roty(phi(2,3))*rotz(phi(3,3));
phi(:,4)=fsolve(@RRR3theta3myfun,[pi/2 -pi/2 -pi/2].');R{4}=rotz(phi(1,4))*roty(phi(2,4))*rotz(phi(3,4));
phi(:,5)=fsolve(@RRR3theta3myfun,[-pi/2 pi/2 pi/2].');R{5}=rotz(phi(1,5))*roty(phi(2,5))*rotz(phi(3,5));
phi(:,6)=fsolve(@RRR3theta3myfun,[-pi/2 pi/2 -pi/2].');R{6}=rotz(phi(1,6))*roty(phi(2,6))*rotz(phi(3,6));
phi(:,7)=fsolve(@RRR3theta3myfun,[-pi/2 -pi/2 pi/2].');R{7}=rotz(phi(1,7))*roty(phi(2,7))*rotz(phi(3,7));
phi(:,8)=fsolve(@RRR3theta3myfun,[-pi/2 -pi/2 -pi/2].');R{8}=rotz(phi(1,8))*roty(phi(2,8))*rotz(phi(3,8));
end

