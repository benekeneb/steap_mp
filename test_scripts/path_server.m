rosshutdown;
rosinit;

server = rossvcserver('/steap_plan', 'carrot_planner/path_array', @serviceCallback,...
                      'DataFormat','struct');

client = rossvcclient('/steap_plan','DataFormat','struct');

req = rosmessage(server);

reqMsg = rosmessage(client);
reqMsg.StartX = 0;
reqMsg.StartY = 0;
reqMsg.GoalX = 0;
reqMsg.GoalY = 0;

global x_array;
global y_array;

[x_array, y_array] = values_to_array(plot_values);

response = call(client,reqMsg,'Timeout',5);

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

