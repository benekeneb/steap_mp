%Its a steap example. Here are few points that needs to be considered 
    % 1. There should be replanning usig Isam
    % 2. Pose estimation should be added to Bayes tree (online thingy)
%{
Following is the overall process of simulataneous trajectpory estimation and planning
    1. There should be a prior (init_values) w.r.t start and goal position
    
    2. FInd the collision-free trajectory from start to end position
    (Trajectory planning step). We'll use the GPMP2 to find this trajectory
    
    3. Now, use the Isam class to generate a Bayes Tree
    
    4. Solve the factor grapoh this time using iSAM. (Note: the batch_values come from step no. 2)
    
    5. Now, the result would be the STEAP problem as discuused in paper.
%}
  
close all
clear

import gtsam.*
import gpmp2.*

debug = 1;

if debug == 0
    %%ROS Config
    rosshutdown
    rosinit

    server = rossvcserver('/steap_plan', 'carrot_planner/path_array', @serviceCallback,...
                          'DataFormat','struct');
    req = rosmessage(server);

    % Arrays that saves current trajectory
    global x_array
    global y_array
end




%% Environment Map
dataset = generate2Ddataset('MobileMap1');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;

%SDF-2D   >> its necessary due to Planar-sdf factors

dataset.origin_x = 0;
dataset.origin_y = 0;

origin_point2 = Point2(dataset.origin_x, dataset.origin_y);
origin_point3 = Point3(dataset.origin_x, dataset.origin_y, 0);

% sdf2D
field2D = signedDistanceField2D(dataset.map, cell_size);
sdf2D = PlanarSDF(origin_point2, cell_size, field2D);

% sdf3D
field3D = signedDistanceField3D(dataset.map, dataset.cell_size);
sdf3D = SignedDistanceField(origin_point3, cell_size, size(field3D, 1), ...
    size(field3D, 2), size(field3D, 3));
for z = 1:size(field3D, 3)
    sdf3D.initFieldData(z-1, field3D(:,:,z)');
end

%% Robot Model and settings parameters
    % Robot model parameters should be changed
total_time_sec = 5.0;
total_time_step = 2;
total_check_step = 2;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use 2d vehicle dynamics
use_vehicle_dynamics = false;
dynamics_sigma = 0.001;

% use GP interpolation
use_GP_inter = false;

% arm model
marm = generateMobileArm('SimpleTwoLinksArm');

% GP
Qc = 1 * eye(5);
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% noise model
pose_fix_sigma = 0.0001; % Note that the noise model for sensor would be most likely different
vel_fix_sigma = 0.0001;

% Obstacle avoid settings
cost_sigma = 0.01;
epsilon_dist = 0.5;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(5, 0.1);
vel_fix = noiseModel.Isotropic.Sigma(5, 0.0001);

% start and end conf
if debug == 0
    [x_ist, y_ist, t_ist] = get_pose_estimate();
else
    x_ist = 2;
    y_ist = 2;
    t_ist = 0;
end
start_pose = Pose2(x_ist, y_ist, t_ist);
start_conf = [0, 0]'; %angle values
pstart = Pose2Vector(start_pose, start_conf);
start_vel = [0, 0, 0, 0, 0]';

end_pose = Pose2(7, 7, 0);
end_conf = [0 0]';
pend = Pose2Vector(end_pose, end_conf);
end_vel = [0, 0, 0, 0, 0]';

avg_vel = [end_pose.x()-start_pose.x(); end_pose.y()-start_pose.y(); ...
    end_pose.theta()-start_pose.theta(); (end_conf / total_time_step)] / delta_t;

% plot param
pause_time = total_time_sec / total_time_step;


%% initial values

init_values = Values;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % initialize as straight line in conf space
    conf = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step;
    pose = Pose2(start_pose.x() * (total_time_step-i)/total_time_step + ...
        end_pose.x() * i/total_time_step, ...
        start_pose.y() * (total_time_step-i)/total_time_step + ...
        end_pose.y() * i/total_time_step, ...
        start_pose.theta() * (total_time_step-i)/total_time_step + ...
        end_pose.theta() * i/total_time_step);
    vel = avg_vel;
    
    insertPose2VectorInValues(key_pos, Pose2Vector(pose, conf), init_values);
    init_values.insert(key_vel, vel);


end
%% build graph
graph = NonlinearFactorGraph;

% key_pos1 = symbol('x', 1);
% key_pos2 = symbol('x', 2);
% key_vel1 = symbol('v', 1);
% key_vel2 = symbol('v', 2);
% graph.add(GaussianProcessPriorPose2Vector(key_pos1, key_vel1,key_pos2,key_vel2, delta_t, Qc_model));

i = 1;

key_pos1 = symbol('x', i-1);
key_pos2 = symbol('x', i);
key_vel1 = symbol('v', i-1);
key_vel2 = symbol('v', i);
graph.add(GaussianProcessPriorPose2Vector(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));


i = 2;

key_pos1 = symbol('x', i-1);
key_pos2 = symbol('x', i);
key_vel1 = symbol('v', i-1);
key_vel2 = symbol('v', i);
graph.add(GaussianProcessPriorPose2Vector(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));

% for i = 0 : total_time_step
%     key_pos = symbol('x', i);
%     key_vel = symbol('v', i);
%     
% %     % start/end priors
% %     if i==0
% %         graph.add(PriorFactorPose2Vector(key_pos, pstart, pose_fix));
% %         graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));      
% %     elseif i==total_time_step
% %         graph.add(PriorFactorPose2Vector(key_pos, pend, pose_fix));
% %         graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
% %     end
% %     
%     % cost factor
% %     graph.add(ObstaclePlanarSDFFactorPose2MobileArm(key_pos, ...
% %         marm, sdf2D, cost_sigma, epsilon_dist));
%     
%     % GP priors and cost factor
%     if i == 1  || i == 2
%     end
% end

%% optimize!
use_trustregion_opt = true;
use_LM_opt = true;

parameters = LevenbergMarquardtParams;
parameters.setVerbosity('ERROR');
optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);


optimizer.optimize();
batch_values = optimizer.values();
