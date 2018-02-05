%% INSTRUCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code by Nicola Bezzo 10/01/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
u(:,1) = zeros(1,1);
y(:,1) = zeros(3,1);
% u(:,1) = [0;1];
 
%%% Use these values for a 4x4 system.  Might not be correct but will run
% u(:,1) = zeros(4,1);
% y(:,1) = zeros(4,1);

count=1;
counter1a=0;
counter2a=0;
counter3a=0;
counter1ok=0;
counter2ok=0;
counter3ok=0;
%% PI parameters 
Kp=5; %increase until oscillations appear 
Ki=0.15;  %increase to reduce error
Kd=0.1; %increase to reduce oscillations
count2=0;
%% KF parameters
sig = 0.02; %Prior Variance
sig_u= 0.01; %Input variance
measurement_sig1 = 0.01; % VARIANCE OF THE MEASUREMENT 1
measurement_sig2 = 0.01 % VARIANCE OF THE MEASUREMENT 2
measurement_sig3 = 0.01; % VARIANCE OF THE MEASUREMENT 3
measurement_sig10 = measurement_sig1;
measurement_sig20 = measurement_sig2;
measurement_sig30 = measurement_sig3;
d1=0;
d2=0;
d3=0;
rv(1)=0.01*randn;
k=1;
integral(1) = 0; 
derivative(1) = 0;
mu = 0; %initial prior
eta = 10; % parameters to speed up convergence, i.e., remove attack faster and make covariance grow faster
%% ATTACK VALUE
attack=pi/6; %attack bias value

%% Begin Simulation

for i=0:Ts:100
    count=count+1;
   count2= count2+1;
   
    %% predict
   x(:,count)=A*x(:,count-1) + B*u(:,count-1);
   y(:,count)=C*x(:,count-1);
   if count2 ==10
       k=k+1;
   rv(k)=0.01*randn; % remember to change this to match the other file % NOISE LEVEL FOR EACH SENSOR
   
   count2=0;
   end
   y(1,count)=y(1,count)+rv(k);
      y(2,count)=y(2,count)+rv(k);
         y(3,count)=y(3,count)+rv(k);


%    if(x(:,count)>5)
%       klkl 
%    end
%% Constant attack inserted at following intervals
   if (i<10) || (i>=20 && i<30) || (i>=40 && i<50) || (i>=60 && i<70) || (i>=80 && i<90)

      
       y(3,count)=y(3,count)+attack;
       %y(1,count)=y(1,count)+attack;
       %y(2,count)=y(2,count)+attack;
       
   end
   sig = sig + sig_u;
   
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

   estimate(count) = x(1,count);%mu;
   x(1,count);
  
   %% Cruise Control using PI controller 
   % mantain angle of pi/3 before 50 iterations
   if i<50
      r(count)= pi/3;

  error(count) = atan2(sin(r(count) - estimate(count)),cos(r(count) - estimate(count)));
  integral(count) = integral(count-1) + error(count)*Ts;
  derivative(count) = (error(count) - error(count-1))/Ts;
  u(1,count) = Kp*error(count) + Ki*integral(count) + Kd*derivative(count);
  if u(1,count)>pi;
      u(1,count)=pi;
  end

  % mantain angle pi/6 above 50 iterations
   else
      r(count)= pi/6;
  
  error(count) = atan2(sin(r(count) - estimate(count)),cos(r(count) - estimate(count)));
  integral(count) = integral(count-1) + error(count)*Ts;
  derivative(count) = (error(count) - error(count-1))/Ts;
  u(1,count) = Kp*error(count) + Ki*integral(count) + Kd*derivative(count);
  if u(1,count)>pi;
      u(1,count)=pi;
  end

  end
  
  
   %% update
   
    [mu, sig] = updatef4(x(1,count), sig, y(1,count), measurement_sig1, y(2,count), measurement_sig2, y(3,count), measurement_sig3);
    x(1,count-1)=mu;
    
    if i==25
        y(1,count)
        y(2,count)
        y(3,count)
        x(1,count)
        mu
        sig
        measurement_sig1
        measurement_sig2
        measurement_sig3
    end
%     
%     if i==51.81
%         y(1,count)
%         y(2,count)
%         y(3,count)
%         x(1,count)
%         mu
%         sig
%         measurement_sig1
%         measurement_sig2
%         measurement_sig3
%     end
    
   
        
  
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
title('ANGLE MEASUREMENTS')
xlabel('time')
ylabel('ANGLE measurements value')
legend('meas_1','meas_2','meas_3')

figure(3)
hold on
plot(x(1,:), 'b-')
plot(r, 'r-')
title('ESTIMATE VS REFERENCE')
xlabel('time')
ylabel('angle')
legend('estimate','reference')
hold off

% figure(44)
% plot (abs(x(1,:)-r), 'k-')
% 
% figure(55)
% plot(rv, 'r-')

