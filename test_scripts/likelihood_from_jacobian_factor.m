import gtsam.*
import gpmp2.*

pose_fix = noiseModel.Isotropic.Sigma(5, 1);

% Linearized Goal Factor for Point x = 7, y = 2
% A is covariance matrix
% b is goal point (actual observed measurement)
% x is point on map
% b&x have form of (x, y, theta, delta1, delta2)
A = eye(5)*10
b = [7  2 -0.00274868  0.00149803 5.13813e-06]*10

% x_vector = zeros(1, 2);

figure;
hold on;

for x = 0:25
   for y = 0:25
       x_adj = x/2.5
       y_adj = y/2.5
       
       x_vector = [x_adj y_adj 0 0 0];
       
       error_function = A * transpose(x_vector) - transpose(b)
       likelihood = 0.5 * transpose(error_function) * inv(pose_fix.covariance) * error_function
       
       
      
       
       plot3(x_adj, y_adj, likelihood, 'o r');
   end
end
view(3)
hold off;
