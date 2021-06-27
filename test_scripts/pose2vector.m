import gtsam.*
import gpmp2.*

start_pose = Pose2(2, 2, pi/2);
start_conf = [0, 0]';
pstart = Pose2Vector(start_pose, start_conf);