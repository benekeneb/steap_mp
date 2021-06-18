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

%% settings
error_mode = 0;

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

%optimization settings
opt_setting = TrajOptimizerSetting(3);
% opt_setting.set_total_step(total_time_step);
% opt_setting.set_total_time(total_time_sec);
% opt_setting.set_conf_prior_model(pose_fix_sigma);
% opt_setting.set_vel_prior_model(vel_fix_sigma);
% 
% opt_setting.set_flag_pos_limit(flag_limit);
% opt_setting.set_flag_vel_limit(flag_limit);
% opt_setting.set_joint_pos_limits_up(joint_limit_vec_up);
% opt_setting.set_joint_pos_limits_down(joint_limit_vec_down);
% opt_setting.set_vel_limits(joint_vel_limit_vec);
% opt_setting.set_pos_limit_thresh(joint_limit_thresh);
% opt_setting.set_vel_limit_thresh(joint_vel_limit_thresh);
% opt_setting.set_pos_limit_model(joint_limit_model);
% opt_setting.set_vel_limit_model(joint_vel_limit_model);
% 
% opt_setting.set_epsilon(epsilon_dist);
% opt_setting.set_cost_sigma(cost_sigma);
% opt_setting.set_obs_check_inter(check_inter);
% 
% opt_setting.set_Qc_model(Qc);
% 
% opt_setting.setDogleg();

% test = ISAM2TrajOptimizer2DArm(robot, sdf, opt_setting);

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

%% optimize!
use_trustregion_opt = true;
use_LM_opt = true;

parameters = LevenbergMarquardtParams;
parameters.setVerbosity('ERROR');
optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters); %% init_values = trajectory prior, straight line from goal to end

optimizer.optimize();
result = optimizer.values();
% result.print('Final results')

%% STEAP
isam = ISAM2;

figure(4)
hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);

xlim([0 10])
ylim([0 10])

%plot initial trajectory
plot_inter = check_inter;
total_plot_step = total_time_step * (plot_inter + 1);
plot_values = interpolatePose2Traj(result, Qc_model, delta_t, plot_inter, 0, total_time_step);
plot_trajectory(plot_values, total_plot_step, 'r');

for i = 0 : total_time_step - 1
    key_pos = symbol('x', i+1);
    goal = result.atPose2(key_pos);
    
    plot_inter = check_inter; %interpolate to next time step
    total_plot_step = total_time_step * (plot_inter + 1);
    exec_values = interpolatePose2Traj(result, Qc_model, delta_t, 5, i, i+1);
    plot_trajectory(exec_values, total_plot_step, 'b');
    
    coll_cost = CollisionCostPose2MobileBase2D(robot, sdf, exec_values, opt_setting); %calculate collision cost
    
    if coll_cost ~= 0 && error_mode == 1
        error("At step %i, plan is not collision free (Collision Cost: %i)", i, coll_cost);
    end
    
    %execute Trajectory
    provide_trajectory(exec_values, 6);
    [x_ist, y_ist, t_ist] = send_goal(goal.x, goal.y, goal.theta);
    
    %get current state and use if it was measured recently then
    %update factor graph to perform incremental inference
    plot(x_ist, y_ist, 'O g');
    
%     pose_estimate = Pose2(x, y, t);
%     estimation_noise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 15]);
%     graph.add(PriorFactorPose2(key_pos, pose_estimate, estimation_noise));
%     
%     %optimize again
%     parameters = LevenbergMarquardtParams;
%     parameters.setVerbosity('ERROR');
%     optimizer = LevenbergMarquardtOptimizer(graph, result, parameters);
%     optimizer.optimize();
%     result = optimizer.values();
%     
%     provide_trajectory(result);
end

%% plot final values
% plot_inter = check_inter;
% if plot_inter
%     total_plot_step = total_time_step * (plot_inter + 1);
%     plot_values = interpolatePose2Traj(result, Qc_model, delta_t, plot_inter, 0, total_time_step);
% else
%     total_plot_step = total_time_step;
%     plot_values = result;
% end
% 
% for i=0:total_plot_step
%     if i>0
%         p0_x = plot_values.atPose2(symbol('x', i)).x;
%         p0_y = plot_values.atPose2(symbol('x', i)).y
%         
%         p1_x = plot_values.atPose2(symbol('x', i-1)).x;
%         p1_y = plot_values.atPose2(symbol('x', i-1)).y;
%         
%         
%     %     plotPlanarMobileBase(robot.fk_model(), p, [0.4 0.2], 'b', 1);
%         plot([p0_x p1_x], [p0_y p1_y], 'g');
%     end
% end

%% FUNCTIONS
%ROS Trajectory Service
function provide_trajectory(Values, steps)
    global x_array;
    global y_array;

    [x_array, y_array] = values_to_array(Values, steps);
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

%Other Functions
function plot_trajectory(values, plot_step, color)
    import gtsam.*;
    import gpmp2.*;
    for i = 0 : plot_step
        if i>0
            try
                p0_x = values.atPose2(symbol('x', i)).x;
                p0_y = values.atPose2(symbol('x', i)).y;

                p1_x = values.atPose2(symbol('x', i-1)).x;
                p1_y = values.atPose2(symbol('x', i-1)).y;

            %     plotPlanarMobileBase(robot.fk_model(), p, [0.4 0.2], 'b', 1);
                plot([p0_x p1_x], [p0_y p1_y], color);
            catch
                break;
            end
        end
    end
end



