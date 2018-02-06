close all
clear all

% chose filename.  Note that this file and the csv file need to be in the
% same directory.  You will need to remove wills additional info on the
% side about time stamps and force to get this to work, along with the
% labels at the top.  Its reccomended you make a new file for this matlab
% code and keep wills raw data for reference.

filename = 'z-point00001-5DATA.csv';
M = csvread(filename);


% pick a time frame from a flat line to the first peak.

time = M(139:148,1); 
q0 = M(139:148,2);
q1 = M(139:148,3);
q2 = M(139:148,4);
q3 = M(139:148,5);

% time = M(327:429,1);
% q0 = M(327:429,2);
% q1 = M(327:429,3);
% q2 = M(327:429,4);
% q3 = M(327:429,5);

quat = [ q0 q1 q2 q3 ];
%roll = x-axis rotation
%pitch = y-axis rotation
%yaw = z-axis rotation
roll = atan2(2*(q0.*q1 +q2.*q3),1-2*(q1.*q1 + q2.*q2)); 
pitch = asin(2*(q0.*q2 - q3.*q1));
yaw = atan(2*(q0.*q3 + q1.*q2)./(1-2*(q2.*q2 + q3.*q3)));  %% or atan2?



% % Parameters to change.  Ts can stay the same,  K1 and I need to be
% varried.

Ts = 1;%0.005; %0.01
count = 1;
K1 = 30;
I = 0.08;

A = [1 K1* Ts; 
      0 1]; 
B = [0 ; Ts/I];
C = [1 0];

m=size(B,2);
p=size(C,1);
n = size(A,1);
Tmax = size(time,1);

% % Set U to whatever your input is.
x = [yaw(1:1); 0];
u = 0.00001;
y(:,1) = yaw(1:1);
% u(:,1) = [0;1];


for i=1:Ts:Tmax-1
    count=count+1;
   x(:,count)=A*x(:,count-1) + B*u;
    y(:,count)=C*x(:,count);
  

end

% Need to automate and possibly get a better fitting metric.  

diff = y.' - yaw;
diffSquare = diff.*diff;
fit = (sum(diffSquare));

% uncomment for visuals.

% figure(1)
% plot(time,yaw)
% title('yaw v time')
% xlabel('time')
% ylabel('yaw angle')
% 
% figure(2)
% hold on
% plot(time,x(1,:))
% title('sim yaw v time')
% xlabel('time')
% ylabel('yaw angle')
