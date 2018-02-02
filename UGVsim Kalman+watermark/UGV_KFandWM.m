% Run LandSahrkSetup.m first
% Enable or diable shield agaist attack by setting shield variable = 1
% (enabled) or =0 (disabled)
% You can change the attack value to any value, see line 50
%

%% SHIELD
shield=1; % 1 ENABLED; 0 DISABLED

clear x
clear u
clear y
close all
x(:,1) = zeros(n,1);
u(:,1) = zeros(2,1);
y(:,1) = zeros(3,1);
count=1;
step=0;
counter1a=0;
counter2a=0;
counter3a=0;
counter1ok=0;
counter2ok=0;
counter3ok=0;

max_s_angle = pi/6;
% random initial configuration of the robot
x_0 = 100;
y_0 = 100;
theta_0 = 2*pi*rand() - pi;
theta = theta_0; % theoretical angle orientation
theta_tot = theta_0; % spoofed angle orientation
x_pos = x_0;
y_pos = y_0;
x_att = x_0;
y_att = y_0;
v = 1; % speed of robot

%create a triangle to represent my robot
robot(:,1) = [0 2 1]';
robot(:,2) = [0 -2 1]';
robot(:,3) = [5 0 1]';

% goal point
goal_point(:,1) = [round(180) round(20)]';
%goal_point(:,2) = [round(rand()*200) round(rand()*200)]';
%goal_point(:,3) = [round(rand()*200) round(rand()*200)]';

%use path matrix to plot the movement of the robot
path = zeros(2,1000);

%% PI parameters 
Kp=3; 
Ki=0;  
Kd=0;
Kv=1;
%count2=0;
%% KF parameters
sig = 0.6; %Prior Variance
sig_u= 0.5; %Input variance
measurement_sig1 = 0.5; % VARIANCE OF THE MEASUREMENT 1
measurement_sig2 = 0.5; % VARIANCE OF THE MEASUREMENT 2
measurement_sig3 = 0.5; % VARIANCE OF THE MEASUREMENT 3
measurement_sig10 = measurement_sig1;
measurement_sig20 = measurement_sig2;
measurement_sig30 = measurement_sig3;
d1=0;
d2=0;
d3=0;
%rv(1)=0.2*randn;
%k=1;
integral(1) = 0; 
mu = 0; %initial prior
eta = 10; % parameters to speed up convergence, i.e., remove attack faster and make covariance grow faster
%% ATTACK VALUE
attack=1; %attack bias value

%% Begin Simulation
for i=0:Ts:4
   count=count+1;
   step = step+1;
   %count2= count2+1;
   %the location of the robot (for plotting):
   A_T_B = [cos(theta) -sin(theta) x_pos; sin(theta) cos(theta) y_pos; 0 0 1];
   robot_pose=A_T_B*robot; %where robot actually is
   A_T_Batt = [cos(theta_tot) -sin(theta_tot) x_att; sin(theta_tot) cos(theta_tot) y_att; 0 0 1];
   robot_pose_att=A_T_Batt*robot; %where robot THINKS it is
    path(1,step) = x_pos;
    path(2,step) = y_pos;
    path_att(1,step) = x_att;
    path_att(2,step) = y_att;

    %% predict
   x(:,count)=A*x(:,count-1) + B*u(:,count-1); %x = [theta; theta_dot]?
   y(:,count)=C*x(:,count-1); %y = 3 sensor values?
   %if count2 ==10
   %    k=k+1;
   %rv(k)=0.2*randn; % remember to change this to match the other file % NOISE LEVEL FOR EACH SENSOR

   %count2=0;
   %end
   y(1,count)=y(1,count)+0.2*randn; % sensor measurement 1 + noise
      y(2,count)=y(2,count)+0.2*randn; % sensor measurement 2 + noise 
         y(3,count)=y(3,count)+0.2*randn; % sensor measurement 3 + noise

   theta = (y(1,count)+y(2,count)+y(3,count))/3; % average measurements to get angle robot should be at

%    if(x(:,count)>5)
%       klkl 
%   end
%% Constant attack inserted at i>10
   if i>1

       y(3,count)=y(3,count)+attack;

   end
   sig = sig + sig_u; %not sure what this is for
   x(1,count)
   theta_tot = (y(1,count)+y(2,count)+y(3,count))/3 % average w/ attacked measurement to get angle robot actually is at

    %% shield
if shield==1    
    if (abs(y(1,count)-mu)/measurement_sig1)>1

        d1=d1+eta*(abs(y(1,count)-mu)/measurement_sig1 - 1);
        measurement_sig1 = measurement_sig1 + d1;    
        counter1a=counter1a+1;
    else

        d1 = d1;
        measurement_sig1 = measurement_sig1;

    end

    if (abs(y(1,count)-mu)/measurement_sig10) <= 1

        measurement_sig1 = measurement_sig10;
        d1=0;
        counter1ok=counter1ok+1;
    end


    if (abs(y(2,count)-mu)/measurement_sig2)>1

        d2=d2+eta*(abs(y(2,count)-mu)/measurement_sig2 - 1);
        measurement_sig2 = measurement_sig2 + d2;    
        counter2a=counter2a+1;
    else

        d2 = d2;
        measurement_sig2 = measurement_sig2;

    end

    if (abs(y(2,count)-mu)/measurement_sig20) <= 1

        measurement_sig2 = measurement_sig20;
        d2=0;
        counter2ok=counter2ok+1;
    end


    if (abs(y(3,count)-mu)/measurement_sig3)>1

        d3=d3+eta*(abs(y(3,count)-mu)/measurement_sig3 - 1);
        measurement_sig3 = measurement_sig3 + d3;    
        counter3a=counter3a+1;
    else

        d3 = d3;
        measurement_sig3 = measurement_sig3;

    end

    if (abs(y(3,count)-mu)/measurement_sig30) <= 1

        measurement_sig3 = measurement_sig30;
        d3=0;
        counter3ok=counter3ok+1;
    end

end

   estimate(count) = theta_tot; %used to be: estimate(count) = x(1,count)
  %% Cruise Control using PI controller
  % mantain speed v=10
%  r(count)= 10;

 % error(count) = r(count) - estimate(count);
  %integral(count) = integral(count-1) + error(count)*Ts;
  %u(1, count) = Kp*error(count) + Ki*integral(count); 
  %if u(1,count)>36;
   %   u(1,count)=36;
 % end
 % u(2,count) = u(1,count);

   v = Kv*sqrt((goal_point(1,1)-x_att)^2+(goal_point(2,1)-y_att)^2); % control speed as a function of distance from goal
   
   if sqrt((goal_point(1,1)-x_att)^2+(goal_point(2,1)-y_att)^2) < 2 % if reached goal, set speed to 0
       v = 0;
   end

   move_step = v * Ts;
   r(count) = atan2(goal_point(2,1)-y_att,goal_point(1,1)-x_att); % desired angle
   error(count) = atan2(sin(r(count) - estimate(count)),cos(r(count) - estimate(count)));
   %error(count) = r(count) - estimate(count); % error in angle
   integral(count) = integral(count-1) + error(count)*Ts;
   derivative(count) = (error(count) - error(count-1))/Ts;
   u(1, count) = Kp*error(count) + Ki*integral(count) + Kd*derivative(count);
   %max steering angle
   %if u(1,count) > max_s_angle
    %   u(1,count) = max_s_angle;
   %elseif u(1,count) < -max_s_angle
    %   u(1,count) = -max_s_angle;
   %end  

   %u(2,count) = u(1,count);

   %% update
    [mu, sig] = updatef4(x(1,count), sig, y(1,count), measurement_sig1, y(2,count), measurement_sig2, y(3,count), measurement_sig3);
    x(1,count-1)=mu;

    %calculate the new value of x and y
    x_pos = x_pos + move_step*cos(theta+u(1,count));
    y_pos = y_pos + move_step*sin(theta+u(1,count));
    x_att = x_att + move_step*cos(theta_tot+u(1,count));
    y_att = y_att + move_step*sin(theta_tot+u(1,count));

    %plot the result:
    plot(x_0,y_0,'o');
    axis([0 200 0 200]);
    hold on;
    plot(goal_point(1,1),goal_point(2,1),'p');
   % plot(goal_point(1,2),goal_point(2,2),'s');
   % plot(goal_point(1,3),goal_point(2,3),'d');
    legend('start point','goal point1'); 
    plot(path(1,1:step),path(2,1:step),'r');
    plot(path_att(1,1:step),path_att(2,1:step),'g');

    title({['x=',num2str(x_pos),'y=',num2str(y_pos)];...
        ['steering angle=',num2str(u(1,count)*180/pi),'speed=',num2str(v)]});
    fill([robot_pose(1,:)],[robot_pose(2,:)],'r');
    fill([robot_pose_att(1,:)],[robot_pose_att(2,:)],'g');
    hold off;
    pause(0.1);
end 
    
figure(1)
plot(u(1,:), 'r')
title('INPUT')
xlabel('time')
ylabel('input value')

figure(2)
hold on 
plot(y(1,:),'r-')
plot(y(2,:),'m-')
plot(y(3,:),'b-')
title('VELCOITY MEASUREMENTS')
xlabel('time')
ylabel('VEL measurements value')
legend('meas_1','meas_2','meas_3')

figure(3)
hold on
plot(x(1,:), 'b-')
plot(r, 'r-')
title('ESTIMATE VS REFERENCE')
xlabel('time')
ylabel('velocity')
legend('estimate','reference')
hold off

% figure(44)
% plot (abs(x(1,:)-r), 'k-')
% 
% figure(55)
% plot(rv, 'r-')

