%% Define model
clear all
close all
m=0.1;
M=2;
l=0.5;
g=9.81;

A=[0 1 0 0; (M+m)*g/(M*l) 0 0 0; 0 0 0 1; -m*g/M 0 0 0];

B=[0; -1/(M*l); 0; 1/M];

C=[1 0 0 0; 0 0 1 0];

D=0;

inv_pend =ss(A,B,C,D);


%% Stability

eignvals = eig(A)

%% Controllability

Cont_M=[B A*B (A^2)*B (A^3)*B]; % You can use ctrb
rank(Cont_M)


%% Pole Placement using given values

P = [-1 -2 -3 -4];

K = place(A,B,P);

sys_cl=ss(A-B*K,B,C,D);

%% LQR 

Q=eye(4);
R =1;
[K_opt P_opt E_opt] =lqr(A,B,Q,R)

sys_cl_opt=ss(A-B*K_opt,B,C,D);


%% comparisson

t = 0:0.1:50;
u = 0*ones(size(t));

x0 = [0;0;5;0];

[Y,T,X]=lsim(sys_cl,u,t,x0);
[Y,T,X_opt]=lsim(sys_cl_opt,u,t,x0);

figure(1)
plot(t,X,'k--',t,X_opt,'r-')

xlabel('time')
ylabel('state')

figure(2)
plot(t,-K(1)*X(:,1)-K(2)*X(:,2),'k--',t,-K_opt(1)*X_opt(:,1)-K_opt(2)*X_opt(:,2),'r-')

xlabel('time')
ylabel('control')