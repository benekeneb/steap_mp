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


%% Environment Map
dataset = generate2Ddataset('MobileMap1');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;

%SDF-2D   >> its necessary due to Planar-sdf factors

origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field2D = signedDistanceField2D(dataset.map, cell_size);
sdf2D = PlanarSDF(origin_point2, cell_size, field2D);

%% Robot Model and settings parameters
    % Robot model parameters should be changed
total_time_sec = 5.0;
total_time_step = 50;
total_check_step = 50;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use 2d vehicle dynamics
use_vehicle_dynamics = true;
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
cost_sigma = 0.1;
epsilon_dist = 0.1;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(5, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(5, 0.0001);

% start and end conf
start_pose = Pose2(-1, 0, pi/2);
start_conf = [0, 0]';
pstart = Pose2Vector(start_pose, start_conf);
start_vel = [0, 0, 0, 0, 0]';

end_pose = Pose2(1, 0, pi/2);
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

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % start/end priors
    if i==0
        graph.add(PriorFactorPose2Vector(key_pos, pstart, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));      
    elseif i==total_time_step
        graph.add(PriorFactorPose2Vector(key_pos, pend, pose_fix));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
    end
    
    % cost factor
    graph.add(ObstaclePlanarSDFFactorPose2MobileArm(key_pos, ...
        marm, sdf2D, cost_sigma, epsilon_dist));
    
    % vehicle dynamics
    if use_vehicle_dynamics
        graph.add(VehicleDynamicsFactorPose2Vector(key_pos, key_vel, ...
            dynamics_sigma));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorPose2Vector(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        
        
        
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPPose2MobileArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    marm, sdf2D, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end

    end
    if i > 1

    end
end

%% plot initial values
% for i=0:total_time_step
%     figure(3), hold on
%     title('Initial Values')
%     % plot world
%     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     % plot arm
%     p = atPose2VectorValues(symbol('x', i), init_values);
%     plotPlanarMobileArm(marm.fk_model(), p, [0.4 0.2], 'b', 2);
%     pause(pause_time), hold off
% end


%% optimize!
use_trustregion_opt = true;
use_LM_opt = true;

if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
elseif use_LM_opt
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end

optimizer.optimize();
batch_values = optimizer.values();


%% The iSAM update w.r.t to the updated factor graph

% Now perform the inference using iSAM update

%optimization settings
opt_setting = TrajOptimizerSetting(5);
opt_setting.set_total_step(total_time_step);
opt_setting.set_total_time(total_time_sec);
opt_setting.set_epsilon(epsilon_dist);
opt_setting.set_cost_sigma(cost_sigma);
opt_setting.set_obs_check_inter(check_inter);
opt_setting.set_conf_prior_model(pose_fix_sigma);
opt_setting.set_vel_prior_model(vel_fix_sigma);
opt_setting.set_Qc_model(Qc);

%{ 
Note that the same above mentioned batch optimization can also be done using the following commands
%}
%init_values = initPose2VectorTrajStraightLine(start_pose, start_conf, end_pose, end_conf, total_time_step);
%batch_values = BatchTrajOptimizePose2MobileArm2D(marm, sdf2D, pstart, start_vel, pend, end_vel, init_values, opt_setting);
    

%{ 
 We also need to have a Bayes tree from the factor graph before update. For
 this purpose we would be using the following class and associated methods
-------Constructors-------
ISAM2TrajOptimizerPose2MobileArm2D(Pose2MobileArmModel marm, PlanarSDF sdf, TrajOptimizerSetting setting)

-------Methods-------
addPoseEstimate(size_t state_idx, Pose2Vector pose, Matrix pose_cov) : returns void
addStateEstimate(size_t state_idx, Pose2Vector pose, Matrix pose_cov, Vector vel, Matrix vel_cov) : returns void
changeGoalConfigAndVel(Pose2Vector goal_conf, Vector goal_vel) : returns void
fixConfigAndVel(size_t state_idx, Pose2Vector conf_fix, Vector vel_fix) : returns void
initFactorGraph(Pose2Vector start_conf, Vector start_vel, Pose2Vector goal_conf, Vector goal_vel) : returns void
initValues(Values init_values) : returns void
removeGoalConfigAndVel() : returns void
update() : returns void
values() : returns gtsam::Values
%}

%initialize the class for Isam
marm_inc_inf = ISAM2TrajOptimizerPose2MobileArm2D(marm, sdf2D, opt_setting);

%Use the above mentioned method to add graph
marm_inc_inf.initFactorGraph(pstart, start_vel, pend, end_vel);
    
% Here the state estimation should be added by using the method as
    %addStateEstimate(size_t state_idx, Pose2Vector pose, Matrix pose_cov, Vector vel, Matrix vel_cov) : returns void
% Note: the values will come from ROS for the real time state estimation

% Update iSAM & get output
marm_inc_inf.initValues(batch_values);
marm_inc_inf.update();
inc_inf_values = marm_inc_inf.values();
%% plot batch values
for i=0:total_time_step
    figure(4), hold on
    title('Initial Optimized Trajectory')
    % plot world
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
    % plot arm
    p = atPose2VectorValues(symbol('x', i), batch_values);
    plotPlanarMobileArm(marm.fk_model(), p, [0.4 0.2], 'b', 1);
    pause(pause_time), hold off
end
%% plot Final Results
% plot the final results that gets updated according to iSAM results