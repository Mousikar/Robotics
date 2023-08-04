function [sys,x0,str,ts,simStateCompliance] = SMC_input(t,x,u,flag,pa)
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
% 输出是：x1d   dotx1d ddotx1d
sizes.NumOutputs     = 3;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
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
A=pa.A;
T=pa.T;
x1d = A*sin(2*pi/T*t);
dx1d= 2*pi/T*A*cos(2*pi/T*t);
ddx1d = -2*pi/T*2*pi/T*A*sin(2*pi/T*t);
sys = [x1d;dx1d;ddx1d];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;  
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];