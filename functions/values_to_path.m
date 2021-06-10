function path = values_to_path(values)

    import gtsam.*
    import gpmp2.*
   
    path_msg = rosmessage('nav_msgs/Path','DataFormat','struct');
    
    [t, issim] = rostime('now','DataFormat','struct');
    path_msg.Header.Stamp.Sec = t.Sec;
%     path_msg.Header.Stamp.Nsec = sec.Nsec;
    path_msg.Header.FrameId = 'map';

       
    end_reached = 0;
    i = 0;
    while end_reached == 0
        try
            values.atPose2(symbol('x', i))
        catch
            end_reached = 1;
            break;
        end
        
        goalMsg = rosmessage('geometry_msgs/PoseStamped','DataFormat','struct');
        goalMsg.Header.FrameId = 'map';
        goalMsg.Header.Stamp.Secs = t.Sec;
        
        euler_vector = zeros(1, 3);
        euler_vector(1) = values.atPose2(symbol('x', i)).theta();
        quaternion_vector = eul2quat(euler_vector);
        orient_x = quaternion_vector(3);
        orient_y = quaternion_vector(2);
        orient_z = quaternion_vector(1);
        orient_w = quaternion_vector(4);
        
        goalMsg.Pose.Position.X = values.atPose2(symbol('x', i)).x();
        goalMsg.Pose.Position.Y = values.atPose2(symbol('x', i)).y();
        goalMsg.Pose.Position.Z = 0;

        goalMsg.Pose.Orientation.X = orient_x;
        goalMsg.Pose.Orientation.Y = orient_y;
        goalMsg.Pose.Orientation.Z = orient_z;
        goalMsg.Pose.Orientation.W = orient_w;

        path_msg.Poses(i + 1) = goalMsg;

        i = i+1;
    end
        
    path = path_msg;
end
   