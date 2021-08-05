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
total_time_step = 25;
total_check_step = 25;
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
Qc = 10 * eye(5);
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% noise model
pose_fix_sigma = 0.01; % Note that the noise model for sensor would be most likely different
vel_fix_sigma = 0.001;

% Obstacle avoid settings
cost_sigma = 0.001;
epsilon_dist = 0.5;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(5, pose_fix_sigma);
vel_fix = noiseModel.Isotropic.Sigma(5, vel_fix_sigma);

% Settings for prior pose --> encourages trajectory to go to goal
prior_pose_fix_sigma = 0.01;
prior_pose_fix = noiseModel.Isotropic.Sigma(5, prior_pose_fix_sigma);

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
global graph;
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
    
    if i < total_time_step 
        key_pos_1 = symbol('x', i)
        prior_pose = atPose2VectorValues(key_pos_1, init_values)
        graph.add(PriorFactorPose2Vector(key_pos, prior_pose, prior_pose_fix))
    end
    
    % cost factor
    graph.add(ObstaclePlanarSDFFactorPose2MobileArm(key_pos, ...
        marm, sdf2D, cost_sigma, epsilon_dist));
    
    % vehicle dynamics
    if use_vehicle_dynamics
        graph.add(VehicleDynamicsFactorPose2Vector(key_pos, key_vel, dynamics_sigma));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorPose2Vector(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
    end
end
global num_factors;
num_factors = graph.nrFactors;

% plot initial values
% for i=0:total_time_step
%     figure(3), hold on
%     title('Initial Values')
%     % plot world
%     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     % plot arm
%     p = atPose2VectorValues(symbol('x', i), init_values);
%     plotPlanarMobileBase(marm.fk_model(), estimation_pose, [0.4 0.2], 'b', 1);
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
figure(1), hold on
% plot world
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);

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

plot_trajectory(batch_values, total_time_step, 'b');


%%STEAP
% for i = 0 : total_time_step - 1
time_sum = 0;
time_iter = 0; 

optimized_values = init_values;
plot_trajectory(optimized_values, total_time_step, 'b');

%SQP OPTIMIZER SETTINGS
sqp_options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp')
A = [];
b = [];
Aeq = [];
beq = [];
nonlcon = [];

global graph_lin;
for i = 2 : total_time_step - 1
    disp("iterate");
    key_pos_0 = symbol('x', i);
    key_vel_0 = symbol('x', i);
    x_0 = atPose2VectorValues(key_pos_0, optimized_values);
    
    x_0_x = x_0.pose.x;
    x_0_y = x_0.pose.y;
    x_0_t = x_0.pose.theta;
    x_0_conf = x_0.configuration;
    x_0_c1 = x_0_conf(1);
    x_0_c2 = x_0_conf(2);
    x_0_array = [x_0_x x_0_y x_0_t x_0_c1 x_0_c2];
    
    if i == 0 || i == total_time_step %skip if iteration is start or goal
%         x_0_solution_pose = Pose2(x_0_x, x_0_y, x_0_t);
%         x_0_solution_config = [x_0_c1, x_0_c2]';
%         x_0_solution_vector = Pose2Vector(x_0_solution_pose, x_0_solution_config);
%         x_solution_vector = atPose2VectorValues(key_pos_0, init_values);
    else
        if i > 2
            key_pos_m2 = symbol('x', i-2);
            key_vel_m2 = symbol('x', i-2);
            x_m2 = atPose2VectorValues(key_pos_m2, optimized_values);

            x_m2_x = x_m2.pose.x;
            x_m2_y = x_m2.pose.y;
            x_m2_t = x_m2.pose.theta;
            x_m2_conf = x_m2.configuration;
            x_m2_c1 = x_m2_conf(1);
            x_m2_c2 = x_m2_conf(2);
            x_m2_array = [x_m2_x x_m2_y x_m2_t x_m2_c1 x_m2_c2];
        end
        if i > 1
            key_pos_m1 = symbol('x', i-1);
            key_vel_m1 = symbol('x', i-1);
            x_m1 = atPose2VectorValues(key_pos_m1, optimized_values);

            x_m1_x = x_m1.pose.x;
            x_m1_y = x_m1.pose.y;
            x_m1_t = x_m1.pose.theta;
            x_m1_conf = x_m1.configuration;
            x_m1_c1 = x_m1_conf(1);
            x_m1_c2 = x_m1_conf(2);
            x_m1_array = [x_m1_x x_m1_y x_m1_t x_m1_c1 x_m1_c2];
        end

        if i < total_time_step - 1
            key_pos_1 = symbol('x', i+1);
            key_vel_1 = symbol('x', i+1);
            x_1 = atPose2VectorValues(key_pos_1, optimized_values);
            
            x_1_x = x_1.pose.x;
            x_1_y = x_1.pose.y;
            x_1_t = x_1.pose.theta;
            x_1_conf = x_1.configuration;
            x_1_c1 = x_1_conf(1);
            x_1_c2 = x_1_conf(2);
            x_1_array = [x_1_x x_1_y x_1_t x_1_c1 x_1_c2];
        end
        if total_time_step - 2
            key_pos_2 = symbol('x', i+2);
            key_vel_2 = symbol('x', i+2);
            x_2 = atPose2VectorValues(key_pos_2, optimized_values);

            x_2_x = x_2.pose.x;
            x_2_y = x_2.pose.y;
            x_2_t = x_2.pose.theta;
            x_2_conf = x_2.configuration;
            x_2_c1 = x_2_conf(1);
            x_2_c2 = x_2_conf(2);
            x_2_array = [x_2_x x_2_y x_2_t x_2_c1 x_2_c2];
        end

        % EXECUTE TRAJECTORY TO VARAIBLE i
        goal = x_m1.pose;
        [x_ist, y_ist, t_ist] = send_goal(goal.x, goal.y, goal.theta, debug);

        % GET POSE ESTIMATE
        estimation_pose = Pose2(x_ist, y_ist, t_ist);
        estimation_config = [0, 0]';
        estimation_vector = Pose2Vector(estimation_pose, estimation_config);

%         plotPlanarMobileBase(marm.fk_model(), estimation_pose, [0.4 0.2], 'b', 1);

        % ADD MEASUREMENT FACTOR
        estimate_cov = noiseModel.Isotropic.Sigma(5, 0.01);
        graph.add(PriorFactorPose2Vector(key_pos_m1, estimation_vector, estimate_cov));

        %LINEARIZE GRAPH
        graph_lin = graph.linearize(optimized_values);

        %PERFORM MESSAGE PASSING
        %GET LINEARIZED FACTORS
        f_gp_m1 = get_gp_factor(i-1);
        f_gp_0 = get_gp_factor(i);
        f_gp_1 = get_gp_factor(i+1);
        f_gp_2 = get_gp_factor(i+2);

        f_o_0 =  get_obs_factor(i);
        f_o_m1 =  get_obs_factor(i-1);
        f_o_1 =  get_obs_factor(i);

        f_m_m1 = get_meas_factor(i-1);
        
        f_p_m1 = get_prior_factor(i-1);
        f_p_0 = get_prior_factor(i);
        f_p_1 = get_prior_factor(i+1);
        

        %retrieve A matrices & b vectors
        f_gp_m1_A = f_gp_m1.getA;
        f_gp_m1_A1 = f_gp_m1_A(:, [1,2,3,4,5]);
        f_gp_m1_A2 = f_gp_m1_A(:, [11,12,13,14,15]);
        f_gp_m1_b = f_gp_m1.getb;

        f_gp_0_A = f_gp_0.getA;
        f_gp_0_A1 = f_gp_0_A(:, [1,2,3,4,5]);
        f_gp_0_A2 = f_gp_0_A(:, [11,12,13,14,15]);
        f_gp_0_b = f_gp_0.getb;

        f_gp_1_A = f_gp_1.getA;
        f_gp_1_A1 = f_gp_1_A(:, [1,2,3,4,5]);
        f_gp_1_A2 = f_gp_1_A(:, [11,12,13,14,15]);
        f_gp_1_b = f_gp_1.getb;

        f_gp_2_A = f_gp_2.getA;
        f_gp_2_A1 = f_gp_2_A(:, [1,2,3,4,5]);
        f_gp_2_A2 = f_gp_2_A(:, [11,12,13,14,15]);
        f_gp_2_b = f_gp_2.getb;

        f_o_m1_A =  f_o_m1.getA;
        f_o_m1_b =  f_o_m1.getb;

        f_o_0_A = f_o_0.getA;
        f_o_0_b = f_o_0.getb;

        f_o_1_A = f_o_1.getA;
        f_o_1_b = f_o_1.getb;

        f_m_m1_A = f_m_m1.getA;
        f_m_m1_b = f_m_m1.getb;

        f_p_m1_A = f_p_m1.getA;
        f_p_m1_b = f_p_m1.getb;
        f_p_0_A = f_p_0.getA;
        f_p_0_b = f_p_0.getb;
        f_p_1_A = f_p_1.getA;
        f_p_1_b = f_p_1.getb;
        
        
        %CALCULATE MESSAGES
        %GP0 to x0
        likelihood_gp_0 = @(x) 0.5 * norm(f_gp_0_A1 * transpose([x(1) x(2) x(3) x(4) x(5)]) + f_gp_0_A2 * transpose([x(6) x(7) x(8) x(9) x(10)]) - f_gp_0_b)^2;
        likelihood_gp_m1 = @(x) 0.5 * norm(f_gp_m1_A1 * transpose([x(1) x(2) x(3) x(4) x(5)]) + f_gp_m1_A2 * transpose(x_m2_array) - f_gp_m1_b)^2;
        likelihood_o_m1 = @(x) 0.5 * norm(f_o_m1_A * transpose([x(1) x(2) x(3) x(4) x(5)]) - f_o_m1_b)^2;
        likelihood_m_m1 = @(x) 0.5 * norm(f_m_m1_A * transpose([x(1) x(2) x(3) x(4) x(5)]) - f_m_m1_b)^2;
        likelihood_p_m1 = @(x) 0.5 * norm(f_p_m1_A * transpose([x(1) x(2) x(3) x(4) x(5)]) - f_p_m1_b)^2;
        if i > 2 % if first 
          m_gp0_x0_min = @(x) likelihood_gp_0([x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10)]) + likelihood_gp_m1([x(1) x(2) x(3) x(4) x(5)]) + ...
                              likelihood_o_m1([x(1) x(2) x(3) x(4) x(5)]) + likelihood_m_m1([x(1) x(2) x(3) x(4) x(5)]) + ...
                              likelihood_p_m1([x(1) x(2) x(3) x(4) x(5)]); 
        else 
          m_gp0_x0_min = @(x) likelihood_gp_0([x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10)]) + ...
                              likelihood_o_m1([x(1) x(2) x(3) x(4) x(5)]) + likelihood_m_m1([x(1) x(2) x(3) x(4) x(5)]) + ...
                              likelihood_p_m1([x(1) x(2) x(3) x(4) x(5)]); 
        end
        
        x_init = [x_m1_x x_m1_y x_m1_t x_m1_c1 x_m1_c2 0 0 0 0 0]; % FIRST 5 VALUES: x_m1, LAST 5 VALUES: x_0
        lb = [-inf -inf -inf 0 0 0 0 0 0 0];
        ub = [inf inf inf 0 0 0 0 0 0 0];        
        m_gp0_x0_min_solution = fmincon(m_gp0_x0_min, x_init, A, b, Aeq, beq, lb, ub, nonlcon, sqp_options);
        
        x_m1_array = x_m1_array + m_gp0_x0_min_solution([1, 2, 3, 4, 5]);
        plot(x_m1_array(1), x_m1_array(2), 'O r');
    
        if i > 2 % if first 
          m_gp0_x0 = @(x) likelihood_gp_0([x_m1_array(1) x_m1_array(2) x_m1_array(3) x_m1_array(4) x_m1_array(5) x(1) x(2) x(3) x(4) x(5)]) + ...
                          likelihood_gp_m1(x_m1_array) +  likelihood_o_m1(x_m1_array) + likelihood_m_m1(x_m1_array) + likelihood_p_m1(x_m1_array); 
        else 
          m_gp0_x0 = @(x) likelihood_gp_0([x_m1_array(1) x_m1_array(2) x_m1_array(3) x_m1_array(4) x_m1_array(5) x(1) x(2) x(3) x(4) x(5)]) + ...
                          likelihood_o_m1(x_m1_array) + likelihood_m_m1(x_m1_array) + likelihood_p_m1(x_m1_array);
        end
        
        %GP1 to x0
        likelihood_gp_1 = @(x) 0.5 * norm(f_gp_1_A1 * transpose([x(1) x(2) x(3) x(4) x(5)]) + f_gp_1_A2 * transpose([x(6) x(7) x(8) x(9) x(10)]) - f_gp_1_b)^2;
        likelihood_gp_2 = @(x) 0.5 * norm(f_gp_2_A1 * transpose([x(6) x(7) x(8) x(9) x(10)]) + f_gp_2_A2 * transpose(x_2_array) - f_gp_2_b)^2;
        likelihood_o_1 = @(x) 0.5 * norm(f_o_1_A * transpose([x(6) x(7) x(8) x(9) x(10)]) - f_o_1_b)^2;
        likelihood_p_1 = @(x) 0.5 * norm(f_p_1_A * transpose([x(6) x(7) x(8) x(9) x(10)]) - f_p_1_b)^2;
        
        if i < total_time_step - 2 % if first 
          m_gp1_x0_min = @(x) likelihood_gp_1([x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10)]) + likelihood_gp_2([x(6) x(7) x(8) x(9) x(10)]) + ...
                              likelihood_o_1([x(6) x(7) x(8) x(9) x(10)]) + likelihood_p_m1([x(6) x(7) x(8) x(9) x(10)]); 
        else 
          m_gp1_x0_min = @(x) likelihood_gp_1([x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10)]) + ...
                              likelihood_o_1([x(6) x(7) x(8) x(9) x(10)]) + likelihood_p_m1([x(6) x(7) x(8) x(9) x(10)]); 
        end                 
        x_init = [0 0 0 0 0 x_1_x x_1_y x_1_t x_1_c1 x_1_c2]; % FIRST 5 VALUES: x_0, LAST 5 VALUES: x_1
        lb = [0 0 0 0 0 -inf -inf -inf 0 0];
        ub = [0 0 0 0 0 inf inf inf 0 0];
        m_gp1_x0_min_solution = fmincon(m_gp1_x0_min, x_init, A, b, Aeq, beq, lb, ub, nonlcon, sqp_options);
        
        x_1_array = x_1_array + m_gp1_x0_min_solution([6, 7, 8, 9, 10]);
        
        if i < total_time_step - 2 % if first 
          m_gp1_x0_min = @(x) likelihood_gp_1([x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10)]) + likelihood_gp_2([x(6) x(7) x(8) x(9) x(10)]) + ...
                              likelihood_o_1([x(6) x(7) x(8) x(9) x(10)]) + likelihood_p_m1([x(6) x(7) x(8) x(9) x(10)]); 
        else 
          m_gp1_x0_min = @(x) likelihood_gp_1([x(1) x(2) x(3) x(4) x(5) x_1_array(1) x_1_array(2) x_1_array(3) x_1_array(4) x_1_array(5)]) + ...
                              likelihood_o_1(x_1_array) + likelihood_p_m1(x_1_array); 
        end  

        %o0 to x0
        m_o0_x0 = @(x) 0.5 * norm(f_o_0_A * transpose([x(1) x(2) x(3) x(4) x(5)]) - f_o_0_b)^2;

        m_prior_x0 = @(x) 0.5 * norm(f_p_0_A * (transpose([x(1) x(2) x(3) x(4) x(5)])) - f_p_0_b)^2;
        
        plot_min_of_function(m_gp0_x0, x_0_array, "r");
        plot_min_of_function(m_gp1_x0, x_0_array, "c");
        plot_min_of_function(m_prior_x0, x_0_array, "g");
        plot_min_of_function(m_o0_x0, x_0_array, "b");

%         % CALCULATE BELIEF
        belief = @(x) m_gp0_x0([x(1)-4 x(2)-4 x(3) x(4) x(5)]) + m_gp1_x0([x(1)-4 x(2)-4 x(3) x(4) x(5)]) + m_prior_x0([x(1) x(2) x(3) x(4) x(5)]) + m_o0_x0([x(1) x(2) x(3) x(4) x(5)]);
%         belief = @(x) m_gp0_x0([x(1) x(2) x(3) x(4) x(5)]) + m_gp1_x0([x(1) x(2) x(3) x(4) x(5)]) + m_o0_x0([x(1) x(2) x(3) x(4) x(5)]);
%         belief = @(x)  m_prior_x0([x(1) x(2) x(3) x(4) x(5)]) + m_o0_x0([x(1) x(2) x(3) x(4) x(5)]) + m_gp0_x0([x(1) x(2) x(3) x(4) x(5)]);
%         belief = @(x)  m_prior_x0([x(1) x(2) x(3) x(4) x(5)]) + m_o0_x0([x(1) x(2) x(3) x(4) x(5)]);
    
        figure(1), hold on
        
        x_init = [x_0_x x_0_y x_0_t x_0_c1 x_0_c2];
        lb = [-10 -10 -inf -inf -inf];
        ub = [10 10 inf inf inf];
        x_0_solution = fmincon(belief, x_init, A, b, Aeq, beq, lb, ub, nonlcon, sqp_options);
        
        x_0_x = x_0_x + x_0_solution(1);
        x_0_y = x_0_y + x_0_solution(2);
        x_0_t = x_0_t + x_0_solution(3);
        x_0_solution_pose = Pose2(x_0_x, x_0_y, x_0_t);
        x_0_solution_config = [x_0_solution(4), x_0_solution(5)]';
        x_0_solution_vector = Pose2Vector(x_0_solution_pose, x_0_solution_config);
        
        %plot likelihoods
        figure(2);
        clf(2);
        view(3)
        hold on;
        
        % PLOT MESSAGES
        fsurf(@(x, y) m_gp0_x0([x y 0 0 0]), [-10 10 -10 10], 'r','MeshDensity',5)
        fsurf(@(x, y) m_gp1_x0([x y 0 0 0]), [-10 10 -10 10], 'c','MeshDensity',5)
        fsurf(@(x, y) m_prior_x0([x y 0 0 0]), [-10 10 -10 10], 'g','MeshDensity',5)
        fsurf(@(x, y) m_o0_x0([x y 0 0 0]), [-10 10 -10 10], 'b','MeshDensity',5);

        %PLOT GP0 TO x0
%         fsurf(@(x, y) likelihood_gp0([x y 0 0 0]), [-10 10 -10 10], 'b','MeshDensity',5);
%         fsurf(@(x, y) likelihood_gp1([x y 0 0 0]), [-10 10 -10 10], 'g','MeshDensity',5)
%         fsurf(@(x, y) likelihood_om1([x y 0 0 0]), [-10 10 -10 10], 'r','MeshDensity',5)
%         fsurf(@(x, y) likelihood_mm1([x y 0 0 0]), [-10 10 -10 10], 'c','MeshDensity',5)
        
        legend
    end
    
    figure(1), hold on
    
% %     plotPlanarMobileBase(marm.fk_model(), x_0_solution_pose, [0.4 0.2], 'b', 1);
    
    optimized_values.erase(key_pos_0);
    insertPose2VectorInValues(key_pos_0, x_0_solution_vector, optimized_values);
    
    p0_x = atPose2VectorValues(symbol('x', i), optimized_values).pose.x();
    p0_y = atPose2VectorValues(symbol('x', i), optimized_values).pose.y();

    p1_x = atPose2VectorValues(symbol('x', i-1), optimized_values).pose.x();
    p1_y = atPose2VectorValues(symbol('x', i-1), optimized_values).pose.y();
    
    plot([p0_x p1_x], [p0_y p1_y], 'g');
    plot(x_0_x, x_0_y, 'O g');
    
    pause(1)
end


time_avg = time_sum / total_time_step;
time_sum;


%% FUNCTIONS
function factor = get_prior_factor(variable)
    global graph_lin;
    factor = graph_lin.at(3*variable + 1);
end
function factor = get_obs_factor(variable)
    global graph_lin;
    if i == 0
        factor = graph_lin.at(3);
    else
        factor = graph_lin.at(3*variable + 2);
    end
end
function factor = get_gp_factor(variable)
    global graph_lin;
    factor = graph_lin.at(3*variable + 3);
end
function factor = get_meas_factor(variable)
    global graph_lin;
    global num_factors;
    factor = graph_lin.at(num_factors + (variable - 1));
end

function plot_min_of_function(function_handle, x_init, color)
    options.Algorithm = 'levenberg-marquardt';
    lb = [-inf -inf -inf 0 0];
    ub = [inf inf inf 0 0];
    min = x_init + lsqnonlin(function_handle, x_init, lb, ub, options);
        
    plot(min(1), min(2), "x " + color);
end

%ROS Trajectory Service
function provide_trajectory(Values, steps, debug)
    if debug == 0
        global x_array;
        global y_array;

        [x_array, y_array] = values_to_array(Values, steps);
    end
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
                p0_x = atPose2VectorValues(symbol('x', i), values).pose.x();
                p0_y = atPose2VectorValues(symbol('x', i), values).pose.y();

                p1_x = atPose2VectorValues(symbol('x', i-1), values).pose.x();
                p1_y = atPose2VectorValues(symbol('x', i-1), values).pose.y();

            %     plotPlanarMobileBase(robot.fk_model(), p, [0.4 0.2], 'b', 1);
                plot([p0_x p1_x], [p0_y p1_y], color);
            catch
                break;
            end
        end
    end
end

function [x_ist, y_ist, t_ist] = send_goal(pos_x, pos_y, euler, debug)
    if debug == 0
        euler_vector = zeros(1, 3);
        euler_vector(1) = euler;
        quaternion_vector = eul2quat(euler_vector);
        orient_x = quaternion_vector(3);
        orient_y = quaternion_vector(2);
        orient_z = quaternion_vector(1);
        orient_w = quaternion_vector(4);

        clear('goalReached', 'status')
        chatpub = rospublisher("/move_base_simple/goal", "geometry_msgs/PoseStamped", "DataFormat", "struct");
        goalMsg = rosmessage(chatpub);

        goalMsg.Pose.Position.X = pos_x;
        goalMsg.Pose.Position.Y = pos_y;
        goalMsg.Pose.Position.Z = 0;

        goalMsg.Pose.Orientation.X = orient_x;
        goalMsg.Pose.Orientation.Y = orient_y;
        goalMsg.Pose.Orientation.Z = orient_z;
        goalMsg.Pose.Orientation.W = orient_w;

        % goalMsg.Header.Seq = 9
        [t, issim] = rostime('now','DataFormat','struct');
        goalMsg.Header.Stamp.Sec = t.Sec;
        goalMsg.Header.Stamp.Nsec = t.Nsec;
        goalMsg.Header.FrameId = 'map';

        send(chatpub, goalMsg);

        pause(4)

        goalReached = 0;
        while goalReached == 0
            pause(1)
            [x_ist, y_ist, t_ist] = get_pose_estimate();

            delta_x = abs(pos_x - x_ist);
            delta_y = abs(pos_y - y_ist);
            delta_t = abs(euler - t_ist);

            if delta_x < (0.2 * pos_x) && delta_y < (0.2 * pos_y)
                goalReached = 1;
            end
            fprintf("Driving\n");
        end


        fprintf("Goal Reached\n");
    else
        dist_max_noise = 0; %maximum noise in meter
        theta_max_noise = 0;
        x_noise = (rand - 0.5) * 2 * dist_max_noise;
        y_noise = (rand - 0.5) * 2 * dist_max_noise;
        t_noise = (rand - 0.5) * 2 * theta_max_noise;
        
        x_ist = pos_x + x_noise;
        y_ist = pos_y + y_noise;
        t_ist = euler + t_noise;
    end
end
   