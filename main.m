%%%%%%%%%%%%% ACS6129 Lab%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialisation

% Clear all
clc;
%% Initialise Model
sslabsc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DO NOT MODIFY ANY LINES ABOVE
%
% Develop your programme below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------------------------------------------------------%
%Author: Awabullah M Syed 
%Student ID: 2002 000 33
%----------------------------------------------------------------%
%% ---------------------Session 1 (Week 5) - System Analysis-------------%

% 2) Modelled System Matrices 
A = [1.4 -0.2 6.7 -5.7 ; -0.6 -4.3 0 0.7 ; 1.1 4.3 -6.7 5.9 ;
    0 4.3 1.3 -2.1]; %Dynamic Matrix
B = [ 0 0 ; 5.7 0 ; 1.1 -3.1 ; 1.1 0 ]; % Control (Input) Matrix 
C = [1 0 0 0 ; 0 1 0 0 ; 0 0 1 -1]; % Sensor (Output) Matrix 
D = [0 0 ; 0 0 ; 0 0]; % Direct Term for Input to Output Matrix 

% Reachability and Observability properties
% -----------------Reachability----------------------------------
Wr = ctrb(A,B); %Reachability Matrix 
reach_check = rank(Wr); %Must equal to 4(n=4) since Matrix A is 4x4 
if reach_check == 4 
    disp('The system is reachable ')
else 
    disp('The system is not reachable')
end
% This system is reachable 
%-----------------Observability----------------------------------
Wo = obsv(A,C); %Observability Matrix 
obser_check = rank(Wo); %Must equal to 4(n = 4) since Matrix A is 4x4 
if obser_check == 4
    disp('The system is observable')
else 
    disp('The system is not observable')
end
%This system is observable 
% 3) Modal Canonical Form representation
sys = ss(A,B,C,D);
csys = canon(sys) %Model Canonical Form
E_value = diag(csys.a); %E.Value / Lambda / mode of the system 
disp('The Eigen Values of the system are '); disp(E_value)
%---------------Stability of modes------------------------------
for i= 1:4 
    mode = E_value(i,1) 
    if mode < 0
        disp('The mode is a stable')  
    else 
        disp('The mode is a unstable')
    end
end 
%----------------Reachability & Observability of each modes --------------

Wo_1 = rank(obsv(csys.a,csys.c)) %Observability of the system
Wr_1 = rank(ctrb(csys.a,csys.b)) %Reachability of the system
sys_minreal = minreal(csys)


%% Session 2 Part I - State feedback and feedforward control

% 1)
% Feedback gain K (0:0.05:8)
t_vec = (0:0.1:10)'; %Time vector for simulation
x0 = [1;0]; %Initial state vector
%---------------------Open Loop----------------------------%
sys_OL = ss(A,B,C,D); %Open Loop Tranfer Function 
%----------Desired EigenValue / Poles-----------------------------%
eig_cl = [-1+0.4i;-1-0.4i;-2;-3] %Desired Poles
K = place(A,B,eig_cl); %Gain Matrix K 

%---------------------- 2) FeedForward gain Kr-----------------%
t_s = (0:0.1:15)';
r = ones(length(t_s),1); %Unit step function 
cr_1 = [C(1,:) + C(3,:)];
cr_2 = [C(2,:)];
Cr = [cr_1 ; cr_2]; %Define Cr 
D_u = [0 0 ; 0 0 ]; %Define D 
sys1 = ss(A,B,Cr,D_u);
kxku = [A B ; Cr D_u]\([zeros(4,2); eye(2)])
k_x = kxku(1:4,:) %extract k_x
k_u = kxku(5:6,:) 
k_r = k_u + K*k_x


%--------------------------Different CL Eigenvalues / Poles--------%
eig_new = [-3+0.8i;-3-0.8i;-5;-8] %Desired Poles
K_new = place(A,B,eig_new); %Gain Matrix K 
kxku_new = [A B ; Cr D_u]\([zeros(4,2); eye(2)])
k_x_new = kxku_new(1:4,:) %extract k_x
k_u_new = kxku_new(5:6,:) %(5:6,:)
k_r_new = k_u_new + K_new*k_x
%% Session 2 Part II - Optimal control and Integral Control
% 1) LQR control
% Design matrices
Q_x = C' * eye(3) * C %Qx as diagonal matrix 
rho = 1;
Q_u = rho * eye(2) %Qu as diagonal matrix
sys_lqr = ss(A,B,eye(4),0)
% State Feedback gain K
K_2 = lqr(sys_lqr,Q_x,Q_u)
%--------------closed loop system eigenvalues-----------------%
P = K_2 / (inv(Q_u)*B.') %P should be greater than 0
eig_value = eig(A - B*K_2) %A_cl = (A - BK)* lambda

% 2) 
% Feedforward gain kr
kxku = [A B ; Cr D_u]\([zeros(4,2); eye(2)])
k_x = kxku(1:4,:) %extract k_x
k_u = kxku(5:6,:) %Extract k_u 
kr = k_u + K_2*k_x

%%
% 3) Integral control
%--------------------desired eigenvalues-------------%
eig_des = [-1+0.4i ; -1-0.4i; -2;-3;-4;-5] %desired Eigenvalues

% augmented matrices
rho = 1;
Q_u = rho * eye(2)
cr_1 = [C(1,:) + C(3,:)];
cr_2 = [C(2,:)];
Cr = [cr_1 ; cr_2]; %Define Cr 

Caug = [Cr zeros(2,2)]
D_u = [0 0 ; 0 0 ];

Aaug = [A zeros(4,2); -Cr zeros(2,2)]
Baug = [B; -D_u]
Daug = D_u
sys_aug = ss(Aaug,Baug,Caug,Daug)
% State feedback gain K
Qx1 = eye(6) %Changing QX to match 6x6
%K_aug = lqr(sys_aug,Qx1,Q_u) %using LQR
K_aug = place(Aaug,Baug,eig_des)
Kaug = K_aug(:,1:4) % Gain K matrix 
Ki = -K_aug(:,5:end) % Integral Control Matrix 
Acl = eig(A - B*Kaug)


%------------Different Eigenvalues, Augmented--------%
eig_des1 = [-3+0.8i;-3-0.8i;-5;-8;-9;-10]
K_aug1 = place(Aaug,Baug,eig_des1)
Kaug1 = K_aug1(:,1:4)
Ki1 = -K_aug1(:,5:end)
Acl1 = eig(A - B*Kaug1)

%% Session 3 Part I - Observer design

% Observer design

% 1)
% desired eigenvalues
desired_eig = [-2+0.4i;-2-0.4i;-4;-6]; %Observer E.values
% Observer gain matrix L
K3 = place(A',C',desired_eig); 
L = K3'             %Gain of the observer 
N = norm(L)
%-------------Different Eigenvalues -------%
desired_eig1 = [-3+0.8i;-3-0.8i;-6;-8] %New E.values of obs
K4 = place (A',C',desired_eig1)
L1 = K4' %New Gain of the observer 
N1 = norm (L1)
%--------------Eigenvalues of closed loop compensated sysed-----%
Closed_eigen = eig(A- L*C) %Observer eigenvalues of 
Closed_eigen2 = eig(A-B*Kaug) %Controller Eigenvalues (Previous Controller used)
%-----------Observer with factor of 100x further left--------%
desired_eig2 = [-200+40i;-200-40i;-400;-600]
K5 = place (A',C',desired_eig2)
L2 = K5'

%% Session 3 Part II - Kalman Filter design
%%%%%%%%%%%%
% KF design
sys_kalman = ss(A, [B B], C, 0);
% Design matrices
Rv = eye(3)
Rw = eye(4)
[kest,L,P] = kalman(sys_kalman,Rw,Rv,0); %L = gain matrix
eig_obs1 = eig(A-L*C)   %not used 
K8 = place(A',C',eig_obs1); %not used 
L6 = K8'    %not used
%----------------- Different design matrices - Measurement noice---%
Rv1 = 50*eye(3);
Io = [200 200 200 200];
Rw1 = diag(Io);
sys_kalman1 = ss(A, [B B], C, 0);
[kest1,L1,P1] = kalman(sys_kalman1,Rw1,Rv1,0);

%-------------------------End of Script--------------------------------%