rosshutdown;
rosinit;

server = rossvcserver('/steap_plan', 'carrot_planner/path_array', @serviceCallback,...
                      'DataFormat','struct');

client = rossvcclient('/test','DataFormat','struct');

req = rosmessage(server);

reqMsg = rosmessage(client);
reqMsg.A = int64(2);
reqMsg.B = int64(1);
reqMsg.C = int64(1);

response = call(client,reqMsg,'Timeout',5)

function resp = serviceCallback(~,req,resp)
    resp.Sum = int64(10);
end
