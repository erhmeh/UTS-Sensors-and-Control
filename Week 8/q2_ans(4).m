%% Define Model

A = [1.5 1.75 1.3
    1 0 0;
    0 1 0;
    ];

B = [1;
    0;
    0;
    ];

C = [0 0 0.75];

X = zeros(3,20);
X(1,1) = 1;
X(2,1) = 1;
X(3,1) = 1;

Y = zeros(1,20);

Q = 1*eye(3);
R = 1;

sys = ss(A,B,C,[], 1);

%% LQR

K = lqr(sys, Q, R);

for k = 1:20
    X(:,k+1) = A*X(:,k) + B*(-1)*K*X(:,k);    

    y(k) = C*X(:,k);
end

figure
hold on
plot(X(1,:))
plot(X(2,:))
plot(X(3,:))
plot(y)
legend('X1', 'X2', 'X3', 'Y')
xlabel('k')


