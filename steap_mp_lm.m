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
Qc = 1 * eye(5);
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% noise model
pose_fix_sigma = 0.0001; % Note that the noise model for sensor would be most likely different
vel_fix_sigma = 0.0001;

% Obstacle avoid settings
cost_sigma = 0.001;
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
    parameters.setVerbosity('SILENT');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
elseif use_LM_opt
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('SILENT');
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('SILENT');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end

optimizer.optimize();
batch_values = optimizer.values();


%% The iSAM update w.r.t to the updated factor graph
figure(4), hold on
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

plot_trajectory(batch_values, total_time_step, 'b')


%%STEAP
% for i = 0 : total_time_step - 1
time_sum = 0;
time_iter = 0; 

optimized_values = Values;
global graph_lin;
for i = 1 : total_time_step
    key_pos_0 = symbol('x', i);
    key_vel_0 = symbol('x', i);
    x_0 = atPose2VectorValues(key_pos_0, batch_values);
    
    x_0_x = x_0.pose.x;
    x_0_y = x_0.pose.y;
    x_0_t = x_0.pose.theta;
    x_0_conf = x_0.configuration;
    x_0_c1 = x_0_conf(1);
    x_0_c2 = x_0_conf(2);
    x_0_array = [x_0_x x_0_y x_0_t x_0_c1 x_0_c2];
    
    if i == 1 || i == total_time_step %skip if iteration is start or goal
        insertPose2VectorInValues(key_pos_0, atPose2VectorValues(key_pos_0, init_values), optimized_values);
        continue
    end
    
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
        x_1 = atPose2VectorValues(key_pos_1, batch_values);
    end
    if total_time_step - 2
        key_pos_2 = symbol('x', i+2);
        key_vel_2 = symbol('x', i+2);
        x_2 = atPose2VectorValues(key_pos_2, batch_values);
        
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
    
    plot(x_ist, y_ist, 'O g');
    plotPlanarMobileBase(marm.fk_model(), estimation_pose, [0.4 0.2], 'b', 1);
    
    % ADD MEASUREMENT FACTOR
    graph.add(PriorFactorPose2Vector(key_pos_m1, estimation_vector, pose_fix));
    
    %LINEARIZE GRAPH
    graph_lin = graph.linearize(batch_values);
    
    %PERFORM MESSAGE PASSING
    m_graph = NonlinearFactorGraph;
    m_key_pos_0 = symbol('x', 0);
    m_key_pos_1 = symbol('x', 1);
    m_key_pos_2 = symbol('x', 2);
    m_key_vel_0 = symbol('v', 0);
    m_key_vel_1 = symbol('v', 1);
    m_key_vel_2 = symbol('v', 2);
    
    m_init_values = Values;
    insertPose2VectorInValues(m_key_pos_0, x_0, m_init_values);
    insertPose2VectorInValues(m_key_pos_1, x_1, m_init_values);
    insertPose2VectorInValues(m_key_pos_2, x_2, m_init_values);
    m_init_values.insert(m_key_vel_0, zeros(4,1));
    m_init_values.insert(m_key_vel_1, zeros(4,1));
    m_init_values.insert(m_key_vel_2, zeros(4,1));
    
    estimate_cov = noiseModel.Isotropic.Sigma(5, 0.25);
    
    m_graph.add(GaussianProcessPriorPose2Vector(m_key_pos_0, m_key_vel_0, m_key_pos_1, m_key_vel_1, delta_t, Qc_model));
    m_graph.add(GaussianProcessPriorPose2Vector(m_key_pos_1, m_key_vel_1, m_key_pos_2, m_key_vel_2, delta_t, Qc_model));
    m_graph.add(PriorFactorPose2Vector(m_key_pos_1, estimation_vector, estimate_cov));
    m_graph.add(ObstaclePlanarSDFFactorPose2MobileArm(m_key_pos_1, marm, sdf2D, cost_sigma, epsilon_dist));
    
    % ADD UNARY FACTORS TO FIX x_0 & x_2
    fix_cov = noiseModel.Isotropic.Sigma(5, 0.000000000001);
    m_graph.add(PriorFactorPose2Vector(m_key_pos_0, x_0, fix_cov));
    m_graph.add(PriorFactorPose2Vector(m_key_pos_2, x_2, fix_cov));
    
    
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
    optimizer = LevenbergMarquardtOptimizer(m_graph, m_init_values, parameters);
    optimizer.optimize(); %needs only to be optimized with respect to 
    optimizer_values = optimizer.values();
    
    x_1_from_message = atPose2VectorValues(symbol('x', 1), optimizer_values)
    
    if i == 2
        break;
    end
end


time_avg = time_sum / total_time_step;
time_sum;


%% FUNCTIONS
function factor = get_obs_factor(variable)
    global graph_lin;
    factor = graph_lin.at((variable - 1) * 2 + 2 + 1)
end

function factor = get_gp_factor(variable)
    global graph_lin;
    factor = graph_lin.at((variable - 1) * 2 + 2 + 2)
end
function factor = get_meas_factor(variable)
    global graph_lin;
    global num_factors;
    factor = graph_lin.at(num_factors + (variable - 1))
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