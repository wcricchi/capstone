clear all
close all
clc


% % % % % % % % % % % Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ts = 0.005;%0.005; %0.01            %timestep

Ac = [1 Ts;
      0 1];
    
Bc = [0;
      1];

Cc = [1 0
      1 0
      1 0];
        
sys_v = ss(Ac,Bc,Cc,0);
sys_vd = c2d(sys_v,Ts);

A = sys_vd.A;
B = sys_vd.B;
C = sys_vd.C;

m=size(B,2);
p=size(C,1);
n = size(A,1)