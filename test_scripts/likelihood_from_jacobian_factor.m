import gtsam.*
import gpmp2.*

% Linearized Goal Factor for Point x = 7, y = 2
% A is covariance matrix
% b is goal point (actual observed measurement)
% x is point on map
% b&x have form of (x, y, theta, delta1, delta2)

% A = eye(5)*10
% b = [7  2 -0.00274868  0.00149803 5.13813e-06]*10
pose_fix = noiseModel.Isotropic.Sigma(5, 1);
cov = pose_fix.covariance;

cov = [002102102121 0 0; 0 002102102121 0; 0 0 002102102121];

A = [3.3333 0 0; 0 3.3333 0; 0 0 10];
b = [3.3333 3.3333 0];
cov = eye(3);

% x_vector = zeros(1, 2);

figure;
hold on;

% for x = 0:50
%    for y = 0:50
minimum = 10000;
x_min = 0;
y_min = 0;
for x = 0:25
   for y = 0:25
       x_adj = x/5
       y_adj = y/5
       
       x_vector = [x_adj y_adj 0];
       
       error_function = A * transpose(x_vector) - transpose(b)
       likelihood_without_cov = 0.5 * transpose(error_function) * error_function
       likelihood_with_cov = 0.5 * transpose(error_function) * inv(cov) * error_function
       
       if likelihood_without_cov ~= likelihood_with_cov
           disp("ERROR");
           break;
       end
       
       if likelihood < minimum
           minimum = likelihood_with_cov;
           x_min = x_adj;
           y_min = y_adj;
       end
       
       plot3(x_adj, y_adj, likelihood_with_cov, '. r');
   end
end

view(3)
hold off;
