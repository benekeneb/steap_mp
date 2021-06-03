function [x, y, t] = get_pose_estimate()
    tftree = rostf;
    pause(1)
    tftree.AvailableFrames;
    pause(1)
    odom_to_base_link = getTransform(tftree, 'odom', 'base_link');
    pause(1)
    x = odom_to_base_link.Transform.Translation.X;
    y = odom_to_base_link.Transform.Translation.Y;
    w = odom_to_base_link.Transform.Rotation.W;
    
    quat_vector = zeros(1, 4);
    quat_vector(4) = w;
    euler_vector = quat2eul(quat_vector);
    t = euler_vector(1);
end