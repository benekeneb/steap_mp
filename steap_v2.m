close all
clear

import gtsam.*
import gpmp2.*

%%ROS Config
rosshutdown
rosinit

server = rossvcserver('/steap_plan', 'carrot_planner/path_array', @serviceCallback,...
                      'DataFormat','struct');
req = rosmessage(server);

% Arrays that saves current trajectory
global x_array
global y_array

%% small dataset
dataset = generate2Ddataset('MobileMap1');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;

dataset.origin_x = 0;
dataset.origin_y = 0;

origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% % plot sdf
% figure(2)
% plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field')


%% settings
total_time_sec = 2.0;
total_time_step = 10; %how many variable factors
check_inter = 5;
delta_t = total_time_sec / total_time_step;
total_check_step = (check_inter + 1)*total_time_step;

% use 2d vehicle dynamics
use_vehicle_dynamics = true;
dynamics_sigma = 0.001;

% robot model
spheres_data = [...
    0  0.0  0.0  0.0  0.8];
nr_body = size(spheres_data, 1);
sphere_vec = BodySphereVector;
for i=1:nr_body
    sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
        Point3(spheres_data(i,2:4)')));
end
robot = Pose2MobileBaseModel(Pose2MobileBase, sphere_vec);

% GP
Qc = 1 * eye(robot.dof());
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% Obstacle avoid settings
cost_sigma = 0.01;
epsilon_dist = 0.2;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(robot.dof(), 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(robot.dof(), 0.0001);

% start and end conf
[x_ist, y_ist, t_ist] = get_pose_estimate();
start_pose = Pose2(x_ist, y_ist, t_ist);
start_vel = [0, 0, 0]';

end_pose = Pose2(8, 5, pi/2);
end_vel = [0, 0, 0]';

avg_vel = [end_pose.x()-start_pose.x(); end_pose.y()-start_pose.y(); ...
    end_pose.theta()-start_pose.theta()] / delta_t;

% plot param
pause_time = total_time_sec / total_time_step;

% % plot start / end configuration
% figure(1), hold on
% plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
% title('Layout')
% plotPlanarMobileBase(robot.fk_model(), start_pose, [0.4 0.2], 'b', 1);
% plotPlanarMobileBase(robot.fk_model(), end_pose, [0.4 0.2], 'r', 1);
% hold off

%% initial values
init_values = Values;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % initialize as straight line in conf space
    pose = Pose2(start_pose.x() * (total_time_step-i)/total_time_step + ...
        end_pose.x() * i/total_time_step, ...
        start_pose.y() * (total_time_step-i)/total_time_step + ...
        end_pose.y() * i/total_time_step, ...
        start_pose.theta() * (total_time_step-i)/total_time_step + ...
        end_pose.theta() * i/total_time_step);
    vel = avg_vel;
    
    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
end

%% build graph
graph = NonlinearFactorGraph;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % start/end priors
    if i==0
        graph.add(PriorFactorPose2(key_pos, start_pose, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
    elseif i==total_time_step
        graph.add(PriorFactorPose2(key_pos, end_pose, pose_fix));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
    end
    
    % cost factor
    graph.add(ObstaclePlanarSDFFactorPose2MobileBase(key_pos, ...
        robot, sdf, cost_sigma, epsilon_dist));
    
    % vehicle dynamics
    if use_vehicle_dynamics
        graph.add(VehicleDynamicsFactorPose2(key_pos, key_vel, ...
            dynamics_sigma));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorPose2(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        
        % GP cost factor
        for j = 1:check_inter
            tau = j * (total_time_sec / total_check_step);
            graph.add(ObstaclePlanarSDFFactorGPPose2MobileBase( ...
                key_pos1, key_vel1, key_pos2, key_vel2, ...
                robot, sdf, cost_sigma, epsilon_dist, ...
                Qc_model, delta_t, tau));
        end
    end
end

%% plot init_values
% figure(5), hold on
% plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
% for i=0:total_time_step
%     p = init_values.atPose2(symbol('x', i));
%     plotPlanarMobileBase(robot.fk_model(), p, [0.4 0.2], 'b', 1);
% end

%% optimize!
use_trustregion_opt = true;
use_LM_opt = true;

parameters = LevenbergMarquardtParams;
parameters.setVerbosity('ERROR');
optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters); %% init_values = trajectory prior, straight line from goal to end


optimizer.optimize();
result = optimizer.values();
% result.print('Final results')

% %% STEAP
figure(4)
hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);

xlim([0 10])
ylim([0 10])

%plot original trajectory
plot_inter = check_inter;
total_plot_step = total_time_step * (plot_inter + 1);
plot_values = interpolatePose2Traj(result, Qc_model, delta_t, plot_inter, 0, total_time_step);

for i=0:total_plot_step
    if i>0
        p0_x = plot_values.atPose2(symbol('x', i)).x;
        p0_y = plot_values.atPose2(symbol('x', i)).y
        
        p1_x = plot_values.atPose2(symbol('x', i-1)).x;
        p1_y = plot_values.atPose2(symbol('x', i-1)).y;
        
        
    %     plotPlanarMobileBase(robot.fk_model(), p, [0.4 0.2], 'b', 1);
        plot([p0_x p1_x], [p0_y p1_y], 'r')
    end
end

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    goal = result.atPose2(key_pos)
    
    goalReached = 0;
    while goalReached == 0
        pause(1)
        [x_ist, y_ist, t_ist] = get_pose_estimate();
        
        delta_x = abs(goal.x - x_ist);
        delta_y = abs(goal.y - y_ist);
        delta_t = abs(goal.theta - t_ist);
        
        if delta_x < (0.15 * goal.x) && delta_y < (0.15 * goal.y)
            goalReached = 1;
        end
        fprintf("Driving\n");
    end
    
    [x, y, t] = get_pose_estimate();
    plot(x, y, 'O g');
    pose_estimate = Pose2(x, y, t);
    estimation_noise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 15]);
    graph.add(PriorFactorPose2(key_pos, pose_estimate, estimation_noise));
    
    %optimize again
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
    optimizer.optimize();
    result = optimizer.values();
    
    % interpolate again
%     total_plot_step = total_time_step * (plot_inter + 1);
%     plot_values = interpolatePose2Traj(result, Qc_model, delta_t, plot_inter, 0, total_time_step);
    
    
   provide_trajectory(result);
end


result.print('Final results')


%% plot final values
plot_inter = check_inter;
if plot_inter
    total_plot_step = total_time_step * (plot_inter + 1);
    plot_values = interpolatePose2Traj(result, Qc_model, delta_t, plot_inter, 0, total_time_step);
else
    total_plot_step = total_time_step;
    plot_values = result;
end

% send_path(plot_values, "/move_base/TebLocalPlannerROS/via_points");

provide_trajectory(plot_values);

for i=0:total_plot_step
    if i>0
        p0_x = plot_values.atPose2(symbol('x', i)).x;
        p0_y = plot_values.atPose2(symbol('x', i)).y
        
        p1_x = plot_values.atPose2(symbol('x', i-1)).x;
        p1_y = plot_values.atPose2(symbol('x', i-1)).y;
        
        
    %     plotPlanarMobileBase(robot.fk_model(), p, [0.4 0.2], 'b', 1);
        plot([p0_x p1_x], [p0_y p1_y], 'g')
    end
end

%% FUNCTIONS

function provide_trajectory(Values)
    global x_array
    global y_array

    [x_array, y_array] = values_to_array(Values)
end

function resp = serviceCallback(~,req,resp)
    global x_array;
    global y_array;
    resp.PathXArray(1) = x_array(1);
    
    i = 1;
    while x_array(i) ~= 0 && y_array(i) ~= 0
        resp.PathXArray(i) = x_array(i);
        resp.PathYArray(i) = y_array(i);
        i = i + 1;
    end
end


