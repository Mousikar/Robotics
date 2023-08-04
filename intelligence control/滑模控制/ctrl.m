function [sys,x0,str,ts,simStateCompliance] = ctrl(t,x,u,flag,pa)
switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u,pa);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;  
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
sys = [];

function sys= mdlUpdate (t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u,pa)
x1d=u(1);
dx1d=u(2);
ddx1d=u(3);
x1=u(4);
x2=u(5);
k=pa.k;
m=pa.m;
p=pa.p;
epsilon=pa.epsilon;
c=pa.c;
e=x1d-x1;
de=dx1d-x2;
s=c*e+de;
M=pa.M;
Delta=pa.Delta;
if M==1
    uc=m*(epsilon*sign(s)+p*s+c*(dx1d-x2)+ddx1d+k/m*x1^3);
elseif M==2
    if abs(s)>Delta
        sat=sign(s);
    else
        sat=s/Delta;
    end
    uc=m*(epsilon*sat+p*s+c*(dx1d-x2)+ddx1d+k/m*x1^3);
end

sys = [uc;e;de];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;  
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];