function theta = subproblem3(p,q,w,delta,r,i)
%subproblem1 子问题3
%   输入：p,q,w,delta,r
%   输出：theta
if nargin==4
    r=[0 0 0].';i=1;
end
u=p-r;
v=q-r;
u1=u-w*(w.'*u);
v1=v-w*(w.'*v);
delta12=delta^2-norm(w.'*(p-q))^2;
theta0=atan2(w.'*cross(u1,v1),u1.'*v1);
theta=theta0+i*acos( ( norm(u1)^2+norm(v1)^2-delta12 )/( 2*norm(u1)*norm(v1) ) );
% theta=theta0-acos( ( norm(u1)^2+norm(v1)^2-delta12 )/( 2*norm(u1)*norm(v1) ) );
end

