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
        dist_max_noise = 0.1; %maximum noise in meter
        theta_max_noise = pi/32;
        x_noise = (rand - 0.5) * 2 * dist_max_noise;
        y_noise = (rand - 0.5) * 2 * dist_max_noise;
        t_noise = (rand - 0.5) * 2 * theta_max_noise;
        
        x_ist = pos_x + x_noise;
        y_ist = pos_y + y_noise;
        t_ist = euler + t_noise;
    end
end
   