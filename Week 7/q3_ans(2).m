% model
A = [0  1; 0 -1];
B = [0; 1];
C = [0 1];
D = 0;
%Open loop model
ex_1 =ss(A,B,C,D);


%% optimal gain for state feedback

Q=[1 0;0 1];
R =1;
[K_opt P_opt E_opt] =lqr(A,B,Q,R)


%P_test_opt=care(A,B,Q,R) %Ricatti equation approach
%K_test_opt = inv(R)*B'*P_opt

test_eig = eig(A-B*K_opt) %Same as E_opt!

% optimal closed-loop system
sys_cl_opt=ss(A-B*K_opt,B,C,D);


%% state feedback using arbitary pole placement
P_old = eig(A)

% new poles
p1 = -1;
p2 = -0.2;

P_sf = [p1 p2];

% use pole placement to get K
K = place(A,B,P_sf);

% arbitary closed-loop system
sys_cl=ss(A-B*K,B,C,D);



%% comparisson
t = 0:0.1:50;
u = 0*ones(size(t));

x0 = [10; -3];

[Y,T,X_opt]=lsim(sys_cl_opt,u,t,x0);
[Y,T,X]=lsim(sys_cl,u,t,x0);

figure(1)
plot(t,X,'k--',t,X_opt,'r-')

xlabel('time')
ylabel('state')

figure(2)
plot(t,-K(1)*X(:,1)-K(2)*X(:,2),'k--',t,-K_opt(1)*X_opt(:,1)-K_opt(2)*X_opt(:,2),'r-')

xlabel('time')
ylabel('control')
