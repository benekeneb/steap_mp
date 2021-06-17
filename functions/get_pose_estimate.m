function [x, y, t] = get_pose_estimate()
    tftree = rostf;
    tftree.AvailableFrames;
    pause(1)
    odom_to_base_link = getTransform(tftree, 'odom', 'base_link');
    pause(1)
    x = odom_to_base_link.Transform.Translation.X;
    y = odom_to_base_link.Transform.Translation.Y;
    orient_x = odom_to_base_link.Transform.Rotation.X;
    orient_y = odom_to_base_link.Transform.Rotation.Y;
    orient_z = odom_to_base_link.Transform.Rotation.Z;
    orient_w = odom_to_base_link.Transform.Rotation.W;
    
    quat_vector = zeros(1, 4);
    quat_vector(1) = orient_x;
    quat_vector(2) = orient_y;
    quat_vector(3) = orient_z;
    quat_vector(4) = orient_w;
    euler_vector = quat2eul(quat_vector);
    t = euler_vector(1);
end