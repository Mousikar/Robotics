function [sys,x0,str,ts,simStateCompliance] = plant(t,x,u,flag,pa)
switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u,pa);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;  
sys = simsizes(sizes);
x0  = [0.5;1];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u,pa)
x1=x(1);
x2=x(2);
k=pa.k;
m=pa.m;
dx1=x2;
dx2=-k/m*x1^3+1/m*u;

sys = [dx1;dx2];

function sys= mdlUpdate (t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
sys = [x(1);x(2)];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;  
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];