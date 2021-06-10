rosinit
server = rossvcserver('/gfdahdqdwq', 'std_srvs/Empty', @exampleHelperROSEmptyCallback,...
                      'DataFormat','struct');
                  
client = rossvcclient('/gfdahdqdwq','DataFormat','struct');

response = call(client)

rosshutdown