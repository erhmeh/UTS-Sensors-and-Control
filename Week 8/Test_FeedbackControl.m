%%
clc;
clear;

%% 
X0 = sqrt(2);
% X0 = -sqrt(2);

%% 
% X0 = sqrt(2)+0.8;
% X0 = -sqrt(2)+0.1;
% X0 = -sqrt(2)+1.8;

%%
n = 100;
X = zeros(n,1);
Xk = X0;


for i=1:n;
    uk = (1+sqrt(2))/2*Xk-(1+sqrt(2))/2;
%     uk = sqrt(2)*Xk-1.5;
%     uk = 0.5;
    Xk1 = FuncNonLinDTS(Xk, uk);
    X(i) = Xk1;
    Xk = Xk1;
end;

%%
figure(1);

plot(1:n,X,'r*-');
% axis equal;


    