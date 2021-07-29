%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief Example of a simple 2D localization example
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Assumptions
%  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
%  - The robot moves 2 meters each step
%  - The robot is on a grid, moving 2 meters each step

%% Create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;

%% Add a Gaussian prior on pose x_1
priorMean = Pose2(1, 1, 0.0); % prior mean is at origin
priorNoise = noiseModel.Diagonal.Sigmas([1; 1; 1]); % 30cm std on x,y, 0.1 rad on theta
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph+

% priorMean = Pose2(1, 1, 0.0); % prior mean is at origin
% priorNoise = noiseModel.Diagonal.Sigmas([1; 1; 1]); % 30cm std on x,y, 0.1 rad on theta
% graph.add(PriorFactorPose2(2, priorMean, priorNoise)); % add directly to graph

edgeMean = Pose2(2, 2, 0.0)
edgeNoise = noiseModel.Diagonal.Sigmas([1; 1; 1]); % 20cm std on x,y, 0.1 rad on theta
graph.add(BetweenFactorPose2(1, 2, edgeMean, edgeNoise));

%% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(1, Pose2(0.0, 0.0, 0.0));
initialEstimate.insert(2, Pose2(0.0, 0.0, 0.0));

initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n  '));

%% Message Passing Algorithm
m_v_to_f = 0;
m_f_to_v = 0;

% VARIABLE TO FACTOR MESSAGES
m_v_to_f = m_f_to_v;

% FACTOR TO VARIABLE MESSAGES
