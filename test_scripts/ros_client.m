

testclient = rossvcclient('/test')

testreq = rosmessage(testclient)
testresp = call(testclient,testreq,'Timeout',3)