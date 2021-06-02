% rosinit

% msg = rosmessage('nav_msgs/OccupancyGrid')
% 
% map_sub = rossubscriber('/map', 'nav_msgs/OccupancyGrid', @msg,"BufferSize",1000000)
% pause(1);
% map_msg = receive(map_sub, 100)
% map = rosReadBinaryOccupancyGrid(map_msg)
% 
% rosshutdown#

master = ros.Core;
% rosinit(getenv('ROS_MASTER_URI'))

% system('roslaunch xyz.launch');
% rosinit
node = ros.Node('/mapReceiver');
% pub = ros.Publisher(node,'/chatter','std_msgs/String','DataFormat','struct');
sub = ros.Subscriber(node,'/map','nav_msgs/OccupancyGrid','DataFormat','struct');
% msg = rosmessage(pub);
% msg.Data = 'hello world';
% send(pub,msg)
% pause(30)
sub.LatestMessage

clear('pub','sub','node')
clear('master')