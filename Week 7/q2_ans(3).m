%% state space model of mass-spring-damper control
clear all
close all

% parameters for the model
m = 1000;
k = 50;
c = 20;

% matrices for the state space model
A = [0  1; -k/m -c/m];
B = [0; 1/m];
C = [1 0];
D = 0;

% define the state space model
spring_damper =ss(A,B,C,D);

% open loop simulation
t = 0:0.1:300;
u = 50*ones(size(t));

% initial state
x0 = [10; -3];

% compute the state and output trajectories
[Y,T,X]=lsim(spring_damper,u,t,x0);

% drawing
figure(1)
plot(t,u)

xlabel('time')
ylabel('input')

figure(2)
plot(t,X(:,1),'r.',t,X(:,2),'k-')

xlabel('time')
ylabel('states')

figure(3)
plot(t,Y)

xlabel('time')
ylabel('output')

%% state feedback using pole placement
% original poles
P_old = eig(A)

 p1 = -0.1;
 p2 = -0.3;

P = [p1 p2]

% use pole placement to get K
K = place(A,B,P)

% closed-loop system
sys_cl=ss(A-B*K,B,C,D);

% check the poles of closed loop system
poles_new = eig(A-B*K)

%% simulation of closed-loop
u = 0*ones(size(t)); 

% compute the state and output trajectories for the closed-loop system
[Y,T,X]=lsim(sys_cl,u,t,x0);

% draw the figures
figure(4)
plot(t,u)

xlabel('time')
ylabel('input')

figure(5)
plot(t,X)

xlabel('time')
ylabel('state')

figure(6)
plot(t,Y)

xlabel('time')
ylabel('output')