import gtsam.*
import gpmp2.*

clear;

rosshutdown;
rosinit;


[x, y, t] = get_pose_estimate();