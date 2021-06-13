%%%%%%%%%%%%% ACS6129 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% State Space Laboratory
%
% Initialisation Programme
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;

%% System Matrices
Ao=[1.38 -0.2077 6.715 -5.676;
   -0.5814 -4.29 0 0.6750;
   1.067 4.273 -6.654 5.893;
   0.048 4.273 1.343 -2.104];
Bo=[0 0; 5.679 0; 1.136 -3.146; 1.136 0];
Co=[1 0 0 0; 0 1 0 0; 0 0 1 -1];
Do=zeros(3,2);

%% Feedforward gain
% Initially identity matrix
kr=[1 0; 0 1];

%% Feedback controller gain - initially no feedback
K=zeros(2,4);

%% Observer gain - initially no feedback of error
%L=zeros(4,3);

% Temporary initialisation to be able to run Simulink
A=Ao;B=Bo;C=Co;D=Do;

% End
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


