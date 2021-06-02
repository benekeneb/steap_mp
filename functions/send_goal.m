function status = send_goal(pos_x, pos_y, orient_w)
    clear('goalReached', 'status')
    chatpub = rospublisher("/move_base_simple/goal", "geometry_msgs/PoseStamped", "DataFormat", "struct");
    goalMsg = rosmessage(chatpub);

    goalMsg.Pose.Position.X = pos_x;
    goalMsg.Pose.Position.Y = pos_y;
    goalMsg.Pose.Position.Z = 0;

    goalMsg.Pose.Orientation.X = 0;
    goalMsg.Pose.Orientation.Y = 0;
    goalMsg.Pose.Orientation.Z = 0;
    goalMsg.Pose.Orientation.W = orient_w;

    % goalMsg.Header.Seq = 9
    [t, issim] = rostime('now','DataFormat','struct');
    goalMsg.Header.Stamp.Sec = t.Sec;
    goalMsg.Header.Stamp.Nsec = t.Nsec;
    goalMsg.Header.FrameId = 'map';

    send(chatpub, goalMsg);
    
    pause(4)

    %%Wait until Goal is reached
    status_sub = rossubscriber('/move_base/status');
    goalReached = 0;
    while goalReached == 0
        pause(1)
        status_msg = receive(status_sub,10);
        status = status_msg.StatusList.Status;
        if status == 3
            goalReached = 1;
        end
        fprintf("Driving\n");
    end

    fprintf("Goal Reached\n");
end
   