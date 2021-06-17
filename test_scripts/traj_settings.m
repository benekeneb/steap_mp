import gtsam.*
import gpmp2.*

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