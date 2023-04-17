function [theta1,theta2] = subproblem2(p,q,w1,w2,r,i)
%subproblem2 子问题二
%   已知p，q，求p到q的转角

    if nargin==4
        r=[0 0 0].';i=1;
    end

u=p-r;
v=q-r;
alpha=( (w1.'*w2)*(w2.'*u)-w1.'*v )/( (w1.'*w2).^2-1 );
beta=( (w1.'*w2)*(w1.'*v)-w2.'*u )/( (w1.'*w2).^2-1 );
gamma2=( norm(u).^2-alpha.^2-beta.^2-2*alpha*beta*(w1.'*w2) )/( norm(cross(w1,w2)).^2 );
% gamma=[sqrt(gamma2);-sqrt(gamma2)];
% z(:,1)=alpha*w1+beta*w2+gamma(1)*cross(w1,w2);
% z(:,2)=alpha*w1+beta*w2+gamma(2)*cross(w1,w2);
% c=z+r;
% theta1=zeros(2,1);
% theta2=zeros(2,1);
% theta1(1)=subproblem1(p,c(:,1),w1);
% theta2(1)=subproblem1(c(:,1),q,w2);
% theta1(2)=subproblem1(p,c(:,2),w1);
% theta2(2)=subproblem1(c(:,2),q,w2);
gamma=i*sqrt(gamma2);
z=alpha*w1+beta*w2+gamma*cross(w1,w2);
c=z+r;
theta2=subproblem1(p,c,w2,r);
theta1=subproblem1(c,q,w1,r);
end

