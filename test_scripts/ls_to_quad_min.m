import gtsam.*
import gpmp2.*

% Linearized Goal Factor for Point x = 7, y = 2
% A is covariance matrix
% b is goal point (actual observed measurement)
% x is point on map
% b&x have form of (x, y, theta, delta1, delta2)

% A = eye(5)*10
% b = [7  2 -0.00274868  0.00149803 5.13813e-06]*10

A = [1 0 0; 0 1 0; 0 0 1];
b = [1 1 0];
b = transpose(b);

figure;
hold on;

minimum = 10000;
x_min = 0;
y_min = 0;
for x = 0:25
   for y = 0:25
       x_adj = x/5;
       y_adj = y/5;
       
       x_vector = [x_adj y_adj 0];
       x_vector = transpose(x_vector)
       
%        error_function = A * transpose(x_vector) - b;
%        likelihood = 0.5 * transpose(error_function) * error_function;

        L = chol(A)
        H = L * transpose(L)

        g = -(transpose(A) * b)
        
%         likelihood = 0.5 * transpose(x_vector) * H * x_vector + transpose(g)*x_vector + 0.5*norm(b)^2
        likelihood = norm(A*x_vector - b)^2
        
       
       if likelihood < minimum
           minimum = likelihood;
           x_min = x_adj;
           y_min = y_adj;
       end
       
       plot3(x_adj, y_adj, likelihood, '. r');
   end
end

disp("Minimum from Graph:")
minimum

%% Convert to Quadratic Minimization Problem using Cholesky Factorization

L = chol(A)
H = L * transpose(L)

g = -(transpose(A) * b)

min = quadprog(H, g, A, b)

likelihood = norm(A*min - b)^2

view(3)
hold off;
