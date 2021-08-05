import gtsam.*
import gpmp2.*

% Linearized Goal Factor for Point x = 7, y = 2
% A is covariance matrix
% b is goal point (actual observed measurement)
% x is point on map
% b&x have form of (x, y, theta, delta1, delta2)

% A = eye(5)*10
% b = [7  2 -0.00274868  0.00149803 5.13813e-06]*10

A = f_p0_A;
b = f_p0_b;

% x_vector = zeros(1, 2);

figure;
hold on;

m_prior_x0 = @(x) 0.5 * norm(f_p0_A * transpose([x(1) x(2) x(3) x(4) x(5)]) - f_p0_b)^2;

% fsurf(@(x, y) m_prior_x0([x y 0 0 0]), [0 10 0 10], 'g','MeshDensity',15)

x0 = [1 2 0 0 0];

lb = [0 0 0 0 -0];
ub = [10 10 0 0 0];

x_0_solution = lsqnonlin(m_prior_x0,x0, lb,ub, options)


view(3)
hold off;
