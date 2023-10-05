classdef RobotMovement < handle

   
     methods (Static)
         function simpleMoveTwoPoints(r, startPos,endPos,count) %r is robot'
             %gets the joint angles
             robotEndEffector = r.model.ikcon(transl(startPos)); %can also be r.model.getpos();
             robotEndEffectorEnd = r.model.ikcon(transl(endPos)*troty(pi));
             %joint angles into jtraj to get positions inbetween
             qMatrix = jtraj(r.model.getpos(),robotEndEffectorEnd,count);
             
                for i = 1:size(qMatrix)
                    r.model.animate(qMatrix(i,:));
                    drawnow();
                    pause(0);
                end
         end

%%function to create the point cloud and find the volume
         function pointCloud = createPointCloud(robot)
             
             stepRads = deg2rad(30);
            qlim = RobotMovement.qlimValues;
            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:4,2)-qlim(1:4,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic
            
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            
                                q5 = 0;
                                q6 = 0;
                                q7 = 0;
            %                     for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q = [q1,q2,q3,q4,q5,q6,q7];
                                    tr = robot.model.fkine(q).T;                        
                                    pointCloud(counter,:) = tr(1:3,4)';
                                    counter = counter + 1; 
                                    if mod(counter/pointCloudeSize * 100,1) == 0
                                        display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                   % end
            %                     end
                            end
                        end
                    end
                end
            end
            
            % 2.6 Create a 3D model showing where the end effector can be over all these samples.  
            pointCloud = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            
            
         end
 
     function radius = drawRadius(robot)
             
            stepRads = deg2rad(5);
            qlim = RobotMovement.qlimValues;
            % Don't need to worry about joint 6
            counter = 1;
             pointCloudeSize = prod(floor((qlim(1:2,2)-qlim(1:2,1))/stepRads + 1));

            pointCloud = zeros(pointCloudeSize,3);

            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    q = [q1,q2, -pi/2, 0, -pi/2, 0 ,0];
                    tr = robot.model.fkine(q).T;                        
                    pointCloud(counter,:) = tr(1:3,4)';
                    counter = counter + 1; 
                                    
                end
            end
            
            % 2.6 Create a 3D model showing where the end effector can be over all these samples.  
            pointCloud = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            radius = 1;

            %calculate volume of robot 
            volume1 =  (pi * radius*radius * 0.8) %calculated from robot arm reachs

            volume2  = (pi * radius*radius*radius * 4/3* 0.25)
            
            
            volumetotal = (volume1+ volume2+ volume2) %2voume twos bc sphere on either end of linear robot

            
         end

    function qlim = qlimValues 
            q1 = [-0.8 -0.01];
            q2567 = deg2rad([-360 360]);
            q3 = deg2rad([-90 90]);
            q4 = deg2rad([-170 170]);
            qlim = [q1; q2567; q3; q4; q2567;q2567;q2567];
    end

     end

end
    
