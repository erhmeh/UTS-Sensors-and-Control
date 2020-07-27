%% Model and plot

d1 = 0.1;
d2 = 0.1;
a1 = 0.6;
a2 = 0.6;
b3 = 0.05;
r3 = 0.8;
x1_0 = 300;
x2_0 = 100;
x3_0 = 80;

Uk = 120;

A = [   1-a1-d1 0   0;
        a1 1-a2-d2 b3;
        0 a2 1-b3-r3;
    ];

B = [1;
    0;
    0;
    ];

C = [0 0 r3];

X = zeros(3,13);

X(1,1) = x1_0;
X(2,1) = x2_0;
X(3,1) = x3_0;

Y = zeros(1,13);

for k = 1:12
    X(:,k+1) = A*X(:,k) + B*Uk;    
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
ylabel('Stock Units')


%% Calculating eigen values of A to test for system stability

eig(A)


%% Test for system controllability

Co = [B A*B A^2*B];
rank(Co)

%% Create discrete time system model

sys = ss(A,B,C,0, 1, 'TimeUnit', 'months');

% Check stability
isstable(sys)

% Calculate control matrix
CO = ctrb(A,B)

% Test number of uncontrollable states
unco = length(A) - rank(CO)

