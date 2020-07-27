clear
clc

fprintf('Problem i');
% u1 = 0, u2 = 1
% x1(k+1) - x1(k) = 0 ==> x1(k)^3 - x1(k) = 0
% x2(k+1) - x2(k) = 0 ==> 2x1(k) + 2x2(k) - 1 = 0

% Solve for x1
x1_ = roots([1 0 -1 0])

% solve for x2
x2_ = zeros(size(x1_));

for i=1:length(x1_)
    x2_(i) = (1 - 2*x1_(i))/2;
end

x1_
x2_

% Linearization
% A =   3x^2    0
%       2       3

% B =   -1       0
%        0      -1

% Calculate A for each equilibrium point (B doesn't change) and check the
% eigen values
for i=1:length(x1_)
    A = [3*x1_(i)^2     0;
        
         2              3;   
        ]
    
    eigenV = eig(A);
    
    fprintf('Eigen values of A for x1 = %d x2 = %d are\n', x1_(i), x2_(i));
    disp(eigenV);
end

%%

fprintf('Problem ii');
% u1 = 3, u2 = 1
% x1(k+1) - x1(k) = 0 ==> 2x1(k)^2 - x1(k) - 3 = 0
% x2(k+1) - x2(k) = 0 ==> 0.5x2(k)^2 - 1.5x2(k) + 1 = 0

% Solve for x1
x1_ = roots([2 -1 -3]);

% solve for x2
x2_ = roots([0.5 -1.5 +1]);

x1_
x2_

% Linerarization
% A =   4x1        0
%         0     -0.5

% Calculate A for each equilibrium point (B doesn't change) and check the
% eigen values
for i=1:length(x1_)
    for j=1:length(x2_)
        A = [4*x1_(i)       0;
             0              x2_(j) - 0.5;            
            ]
        
        eigenV = eig(A);
        fprintf('Eigen values of A for x1 = %d x2 = %d are\n', x1_(i), x2_(j));
        disp(eigenV);
    end
end

%%

fprintf('Problem iii')
% u1 = 0 and u2 = 2
% x1(k+1) - x1(k) = 0 ==> x1(k)^2 = 0
% x2(k+1) - x2(k) = 0 ==> sin(x1(k)) + x2(k)^2 - x2(k) - 2 = 0

% Solve for x1
x1_ = roots([1 0 0]);

% Solve for x2
x2_ = zeros(length(x1_),2);

for i=1:length(x1_)
    x2_(i,:) = roots([1 -1 (sin(x1_(i))-2)]);
end

x1_
x2_

% Linerarization
% A =   2x1+1      0
%       cos(x1)  2x2

% Calculate A for each equilibrium point (B doesn't change) and check the
% eigen values
for i=1:length(x1_)
    for j=1:length(x2_)
        A = [(2*x1_(i)+1)       0;
             cos(x1_(i))        2*x2_(j,1);            
            ]
        
        eigenV = eig(A);
        fprintf('Eigen values of A for x1 = %d x2 = %d are\n', x1_(i), x2_(j,1));
        disp(eigenV);
        
        A = [(2*x1_(i)+1)       0;
             cos(x1_(i))        2*x2_(j,2);            
            ]
        
        eigenV = eig(A);
        fprintf('Eigen values of A for x1 = %d x2 = %d are\n', x1_(i), x2_(j,2));
        disp(eigenV);
    end
end





































