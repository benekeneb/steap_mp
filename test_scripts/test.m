import gtsam.*
import gpmp2.*

pose_fix = noiseModel.Isotropic.Sigma(5, 0.0001);

A = eye(5)*10000
b = [ -0.000749284 -1.65548e-05  0.000179779 -0.000202091 -8.43393e-07 ]

figure;
hold on;

for x = 0:10
   for y = 0:10
       x_vector = [x y 0 0 0];
       
       error_function = A * transpose(x_vector) - b
       likelihood = 0.5 * transpose(error_function) * inv(pose_fix.covariance) * error_function
       
       plot3(x, y, factor_value, 'o');
   end
end

hold off;
