clear all
close all
clf
clc

%!!! set as 1 to run section 2 (point cloud, volume, operating area)
i = 0;

% Environment Creation
Environment.createEnvironment();
hold on;
bPositions = Brick.PlaceBricks();

%%creating the linear ur3robot
r = LinearUR3();
r.model;
% r.TestMoveJoints
q = zeros(1,7);

%%createing the gripper
% g = Gripper();
% g.model;

if i == 1
    pointCloud = RobotMovement.createPointCloud(r);
    pause;
    delete(pointCloud);
    
    robotReach = RobotMovement.drawRadius(r);
    pause;
    delete(robotReach);
end

count = 200;
pause;
%creating brick positions for robot to go to
brickStartPoses = [0.4, 0.4, 1;
                    0.3, 0.4, 1;
                    0.2, 0.4, 1;
                    0.1, 0.4, 1;
                    0, 0.4, 1;
                    -0.1, 0.4, 1;
                    -0.2, 0.4, 1;
                    -0.3, 0.4, 1;
                   -0.4, 0.4, 1;-0.4, 0.4, 1];


brickEndPoses = [-0.3, -0.4, 1;
                -0.2, -0.4, 1;
                -0.1, -0.4, 1;
                0, -0.4, 1;
                -0.3, -0.4, 1.05;
                -0.2, -0.4, 1.05;
                -0.1, -0.4, 1.05;
                -0.1, -0.4, 1.1;
                -0.2, -0.4, 1.1;];

%loop iterating through robots movements and moving bricks
for i = 1:9
      delete(bPositions(i));
     RobotMovement.simpleMoveTwoPoints(r, brickStartPoses(i),brickEndPoses(i),count);
     PlaceObject('HalfSizedRedGreenBrick.ply', brickEndPoses(i,:));
     RobotMovement.simpleMoveTwoPoints(r, brickEndPoses(i),brickStartPoses(i+1),count);
   
end


