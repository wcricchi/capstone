clear all
close all
clc


% % % % % % % % % % % Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m_t  = 232.693;     % 0.8;         %kg                 Mass of the tank
Jt = 26.634538;      % 5*10^(-4);     % kg · m2           Angular mass of the tank
Bv = 0.5557;        % 0.1;            % m                 Width of the tank from track to track
Br = 0.1 ;          % 1.0;           % N · s / m         Mechanical resistance of the tracks to rolling forward
Bs = 0;             % 14.0;          % N · s / m         Mechanical resistance of the tracks to sliding forward
Bl = 0.7;           % N · m · s / rad   Mechanical resistance of the tracks to turning
Sl = 0.57;          % 0.3;           % N · m             Lateral friction of the tracks

Lm = 0.000019 ;     % 10^(-3);       % H                 Inductance of the motor
Rm = 0.18;          % 3.1;           % Ohm               Resistance of the motor
Jg = 0.000018;      % 1.2*10^(-6);   % kg · m2           Angular mass of the gears
Bg = 6.7*10^(-7);   % N · m · s / rad   Mechanical resistance of the gears to rotation
g = 3.636363;       % 204;            %                   Gear ratio of the gearbox
alph = 0.0194421;   % 10^(-3);     % N · m / A         Current–torque ratio of the electric motor
r = 0.1539;         % 0.015;          % m                 Radius of the sprocket wheel
Kt = 1.5;           % 10^(-3);       % m / N             Compliance of the track



Ts = 0.005;%0.005; %0.01
K1 = 1;
K2 = 1;
  
% non-turning
% % % sys_v = ss(-Br/m_t,2/m_t,1,0);
% Ac = [ -(Br+Bs)/m_t     0       0       1/m_t       0       0       1/m_t;
%         0               -Rm/Lm  -alph/Lm  0         0       0        0  ;
%         0               alph/Jg  -Bg/Jg   -r/(Jg*g) 0       0        0  ;
%         -1/Kt           0       r/(Kt*g)  0         0       0        0  ;
%         0               0       0         0         -Rm/Lm  -alph/Lm  0  ;
%         0               0       0         0         alph/Jg -Bg/Jg    -r/(g*Jg)  ;
%         -1/Kt           0       0         0         0       r/(Kt*g)  0     ];
%     
% 
% 
% Bc = [0 0; 1/Lm 0; 0 0; 0 0; 0 1/Lm; 0 0; 0 0];
% 
% 
% Cc = [  1 zeros(1,6);
%         0.98 zeros(1,6);
%         1.03 zeros(1,6); ];

% % % % % For a 2x2 dummy system.  Need to fix values
% %  Ac and Cc are fine for a position/velocity system where we measure
% %  velocity, unsure of what Bc should be  

Ac = [1 K1* Ts; 
      0 1]; 
Bc = [0 ; K2* Ts];
Cc = [1 0];

%%%%%% For a 4x4 dummy system.  Need to look into quaternions more

% w1 = 1;
% w2 = 1;
% w3 = 1;
% 
% Ac = [ 0 -w1 -w2 -w3;
%        w1 0  -w3 -w2;
%        w2 w3  0  -w1;
%        w3 w2 w1   0  ];
% Bc = [1 0 0 0;
%       0 1 0 0;
%       0 0 1 0;
%       0 0 0 1 ];
% Cc = [1 0 0 0;
%       0 1 0 0;
%       0 0 1 0;
%       0 0 0 1 ];
    
        
sys_v = ss(Ac,Bc,Cc,0);
sys_vd = c2d(sys_v,Ts);

% % % Kp=6; Kp_s=50;
% % % Ki=20;  Ki_s=0.5;
% % % Kd=0;
% % % N=100;

Kp=11; Kp_s=50;
Ki=0.2;  Ki_s=0.5;
Kd=0;
N=100;

awgn_level = 0.002;%0*0.002;  % 0, 0.02 , 0.1 ----> 0, 1, 2
pulse_attack = 1*3; %ATTACK



% global Ymem 
% global T
% global C
% global A

A = sys_vd.A;
B = sys_vd.B;
C = sys_vd.C;

% Anew = [A                           B*[1 1]';
%         -1/3*[Kp_s Kp_s Kp_s]*C   Ki_s]

% Anew = [A                           B*[1]';
%         -1/3*[Kp_s]*C   Ki_s]
% abs(eig(A))
% 
% abs(eig(Anew))


m=size(B,2);
p=size(C,1);
n = size(A,1);


% T = 4;
% Ymem = zeros(p,T);
% CAks = zeros(p,T*n);
% Aks = zeros(n,(T-1)*n);
% for ii=1:T
%     CAks(:,(ii-1)*n+1:ii*n) = C*A^(ii-1); 
%     Aks(:,(ii-1)*n+1:ii*n) = A^(ii); 
% end
% % for ii=0:T-1
% %     eval(sprintf('params.CAs_%d = CAks(:,ii*n+1:(ii+1)*n)',ii))
% %     eval(sprintf('params.y_%d = Y(:,ii+1:ii+1)',ii))
% % end
% 
% break
% x0 = [11.1 rand(1,n-1)]';
% 
% Ak=eye(size(A,1));
% Y = zeros(p,T);
% % Y(:,1) = C*x0;
% 
% CAks = zeros(p,T*n);
% % CAks(:,1:n) = C;
% 
% 
% % Reset seed for repeatable tests
% 
% s = RandStream('mcg16807','Seed', 3); % set the random seed
% RandStream.setGlobalStream(s);
%  
% % to create an N-deminsional noise with variance "V" and mean "m"
% V=1*0.05; n_mean=0;
% % noise = sqrtm(V)*rand(N,1) + n_mean
% 
% x=x0;
% for ii=1:T
%     Y(:,ii) = C*x+ [0 6*rand(1) 0*rand(1)]';
%     x=A*x + sqrtm(V)*rand(n,1) + n_mean;
%     CAks(:,(ii-1)*n+1:ii*n) = C*A^(ii-1);
%     
% end
% 
% Y
% CAks
% 
% for ii=0:T-1
%     eval(sprintf('params.CAs_%d = CAks(:,ii*n+1:(ii+1)*n)',ii))
%     eval(sprintf('params.y_%d = Y(:,ii+1:ii+1)',ii))
% end
% 
% 
% tic
% cvx_begin
%     variable x(n)
%     variable PHI_x(p,T)
%     variable M(p,T)
%     variable M_lr(p)
% % 
% %     minimize (norm(sum((Y-PHI_x).^2,2),1))
%     minimize sum(M_lr)
% %     
%     for ii=1:T
%         PHI_x(:,ii) == C*A^(ii-1)*x;
%     end
% %     
%     M == (Y-PHI_x)
%     for jj=1:p
%         M_lr(jj) >= norm(M(jj,:),2)
%     end
% 
% %     minimize norm(Y-PHI_x,1)
% 
% cvx_end
% time_cvx = toc
% 
% % tic
% % cd('cvxgen')
% % [vars, status] = csolve(params);
% % cd('..');
% % 
% % time_c = toc
% 
% x0
% x
% % vars.x
% 
% 
% % % Simulation parameters
% % sim_time = 50;          % length of simulation of each test
% % num_test = 1;          % number of tests to be performed
% % 
% % smart_attack = 1;       % 0 = NO SMART ATTACK; 1 = SMART ATTACK
% % attack_lim = [15 15];    % attack value (assuming constant for all tests)
% % attack_num = 1;         % must be less than "ceil(M/2)-1"
% % attack_time = 50;       % time when an attack begins
% % 
% % % system dynamics
% % a_lim = [1.8 1.8];        % bounds on the single state
% % b_lim = [1 1];         % bounds on the single input
% % x0 = 0;                 % inital condition on state
% % 
% % % measurement dynamics
% % num_sens = 3;          % number of sensors
% % c_lim = [1 1];        % bounds on the measurement state gain
% % 
% % % system noise parameters
% % W = .1;                     % process noise covariance;
% % V = 10*eye(num_sens);        % measurement noise covariance
% % 
% % % Deterministic Estimation Parameters
% % DRSE_win = 5;          % window of the Deterministic estimator
% % 
% % % Nico's Estimator
% % NICO_eta = 10;
% % NICO_sig_x0 = 1;
% % NICO_est_x0 = 15;
% % 
% % 
% % % Simulation parameters
% % sim_time = 50;          % length of simulation of each test
% % num_test = 1;          % number of tests to be performed
% % 
% % smart_attack = 0;       % 0 = NO SMART ATTACK; 1 = SMART ATTACK
% % attack_lim = [15 15];    % attack value (assuming constant for all tests)
% % attack_num = 1;         % must be less than "ceil(M/2)-1"
% % attack_time = 10;       % time when an attack begins
% % 
% % % system dynamics
% % a_lim = [1.8 1.8];        % bounds on the single state
% % b_lim = [0 0];         % bounds on the single input
% % x0 = 0;                 % inital condition on state
% % 
% % % measurement dynamics
% % num_sens = 3;          % number of sensors
% % c_lim = [1 1];        % bounds on the measurement state gain
% % 
% % % system noise parameters
% % W = .1;                     % process noise covariance;
% % V = 10*eye(num_sens);        % measurement noise covariance
% % 
% % % Deterministic Estimation Parameters
% % DRSE_win = 5;          % window of the Deterministic estimator
% % 
% % % Nico's Estimator
% % NICO_eta = 10;
% % NICO_sig_x0 = 1;
% % NICO_est_x0 = 15;
% % 
% % 
