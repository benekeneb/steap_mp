rosinit

% tf_sub = rossubscriber('/tf');
% pause(1);
% tf_msg = receive(tf_sub,10)
% translation = tf_msg.Transforms.Transform.Translation
% rotation = tf_msg.Transforms.Transform.Rotation

tftree = rostf
pause(2);
tftree.AvailableFrames
odom_to_base_link = getTransform(tftree, 'odom', 'base_link');
translation = odom_to_base_link.Transform.Translation
rotation = odom_to_base_link.Transform.Rotation %can be converted to deg easily

rosshutdown