close all
clear

import gtsam.*
import gpmp2.*

%%ROS Config
% rosshutdown
% rosinit
% 
% server = rossvcserver('/steap_plan', 'carrot_planner/path_array', @serviceCallback,...
%                       'DataFormat','struct');
% req = rosmessage(server);

% Arrays that saves current trajectory
global x_array
global y_array

%% generate map & sdf
dataset = generate2Ddataset('MobileMap1');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;

origin = [0, 0, 0];
origin_point3 = Point3(origin');

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% init sdf
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

%% generate mobile arm model
% base_T_arm = Pose3(Rot3(eye(3)), Point3([0,0,0]'));
% arm: JACO2 6DOF arm
% alpha = [0]';
% a = [0]';
% d = [0]';
% % theta = [0, 0, 0, 0, 0, 0]';
% % abstract arm
% arm = Arm(0, a, alpha, d);
% % abstract mobile arm
% marm = Pose2MobileArm(arm, base_T_arm);
% % sphere data [id x y z r]
% spheres_data = [...
% 0  0.0  0.0  0.0  0.8
%     ];
% 
% nr_body = size(spheres_data, 1);
% 
% sphere_vec = BodySphereVector;
% for i=1:nr_body
%     sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
%         Point3(spheres_data(i,2:4)')));
% end
% arm_model = generateMobileArm('');
% arm model
arm_model = generateMobileArm('SimpleTwoLinksArm');

% GP
Qc = 1 * eye(5);
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% Obstacle avoid settings
cost_sigma = 0.1;
epsilon_dist = 0.1;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(5, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(5, 0.0001); 

%% settings
error_mode = 0;

total_time_sec = 2.0;
total_time_step = 10; %how many variable nodes
check_inter = 5;
delta_t = total_time_sec / total_time_step;
total_check_step = (check_inter + 1)*total_time_step;

% use 2d vehicle dynamics
use_vehicle_dynamics = true;
dynamics_sigma = 0.001;

% start and end conf
% [x_ist, y_ist, t_ist] = get_pose_estimate();
start_vector = [2, 2, 0];
start_pose = Pose2(2, 2, 0);
start_vel = [0, 0, 0];
start_conf = Pose2Vector(start_pose, zeros(3,1));

end_vector = [8, 5, pi/2];
end_pose = Pose2(8, 5, pi/2);
end_vel = [0, 0, 0]';
end_conf = Pose2Vector(end_pose, zeros(3,1));

avg_vel = [end_pose.x-start_pose.x; end_pose.y-start_pose.y; ...
    end_pose.theta-start_pose.theta] / delta_t;

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

%% initialize ISAM

%init traj
init_values = initPose2VectorTrajStraightLine(start_pose, zeros(3,1), end_pose, zeros(3,1), total_time_step);
batch_values = BatchTrajOptimizePose2MobileArm(arm_model, sdf, start_conf, zeros(3,1), end_conf, zeros(3,1), init_values, opt_setting)

marm_inc_inf = ISAM2TrajOptimizerPose2MobileArm(arm_model, sdf, opt_setting);
marm_inc_inf.initFactorGraph(start_conf, zeros(3,1), end_conf, zeros(3,1));
marm_inc_inf.initValues(batch_values);
marm_inc_inf.update();

inc_inf_values = marm_inc_inf.values()
 
% %% STEAP
% figure(4)
% hold on
% plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
% 
% xlim([0 10])
% ylim([0 10])
% 
% %plot initial trajectory
% plot_inter = check_inter;
% total_plot_step = total_time_step * (plot_inter + 1);
% plot_values = interpolatePose2Traj(inc_inf_values, Qc_model, delta_t, plot_inter, 0, total_time_step);
% plot_trajectory(plot_values, total_plot_step, 'r');
% 
% for i = 0 : total_time_step - 1
%     key_pos = symbol('x', i+1);
%     goal = inc_inf_values.atPose2(key_pos);
%     
%     plot_inter = check_inter; %interpolate to next time step
%     total_plot_step = total_time_step * (plot_inter + 1);
%     exec_values = interpolatePose2Traj(inc_inf_values, Qc_model, delta_t, 5, i, i+1);
%     plot_trajectory(exec_values, total_plot_step, 'b');
%     
%     coll_cost = CollisionCostPose2MobileBase2D(robot, sdf, exec_values, opt_setting); %calculate collision cost
%     
%     if coll_cost ~= 0 && error_mode == 1
%         error("At step %i, plan is not collision free (Collision Cost: %i)", i, coll_cost);
%     end
%     
%     %execute Trajectory --> Send it to ROS
%     provide_trajectory(exec_values, 6);
%     [x_ist, y_ist, t_ist] = send_goal(goal.x, goal.y, goal.theta);
%     
%     %get current state and use if it was measured recently then
%     %update factor graph to perform incremental inference
%     plot(x_ist, y_ist, 'O g');
%     
%     pose_estimate = Pose2(x, y, t);
%     estimation_noise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 15]);
%     marm_inc_inf.addPoseEstimate(i + 1, pose_estimate, estimation_noise);
%     marm_inc_inf.update();
%     inc_inf_values = marm_inc_inf.values();
% end
% 
% %% FUNCTIONS
% %ROS Trajectory Service
% function provide_trajectory(Values, steps)
%     global x_array;
%     global y_array;
% 
%     [x_array, y_array] = values_to_array(Values, steps);
% end
% 
% function resp = serviceCallback(~,req,resp)
%     global x_array;
%     global y_array;
%     resp.PathXArray(1) = x_array(1);
%     
%     i = 1;
%     while x_array(i) ~= 0 && y_array(i) ~= 0
%         resp.PathXArray(i) = x_array(i);
%         resp.PathYArray(i) = y_array(i);
%         i = i + 1;
%     end
% end
% 
% %Other Functions
% function plot_trajectory(values, plot_step, color)
%     import gtsam.*;
%     import gpmp2.*;
%     for i = 0 : plot_step
%         if i>0
%             try
%                 p0_x = values.atPose2(symbol('x', i)).x;
%                 p0_y = values.atPose2(symbol('x', i)).y;
% 
%                 p1_x = values.atPose2(symbol('x', i-1)).x;
%                 p1_y = values.atPose2(symbol('x', i-1)).y;
% 
%             %     plotPlanarMobileBase(robot.fk_model(), p, [0.4 0.2], 'b', 1);
%                 plot([p0_x p1_x], [p0_y p1_y], color);
%             catch
%                 break;
%             end
%         end
%     end
% end
