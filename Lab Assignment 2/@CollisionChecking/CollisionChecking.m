classdef CollisionChecking 
   %Class for ensuring collisions do not occur 
   %in the brief it says we make and control the object so the easiest
   %would be a sphere just sphere centre and radius if in that volume is a
   %collision 

    properties
        Property1
    end

    methods
      

%Below is all bad code from w5 thinking we can make a combination of q1 and
%3 to check if the robots path will pass through the voloume of the object
%at any step 

%% CheckCollision
% Checks for collisions with an object and can be modified to return an
% isCollision result

%this function is from w5 starter 
        function CheckCollision(robot, sphereCenter, radius)
        % function isCollision = CheckCollision(robot, sphereCenter, radius)

            tr = robot.fkine(robot.getpos).T;
            endEffectorToCenterDist = sqrt(sum((sphereCenter-tr(1:3,4)').^2));
            if endEffectorToCenterDist <= radius
                disp('Oh no a collision!');
        %         isCollision = 1;
            else
                disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
        %         isCollision = 0;
            end
        
        end
        %% IsCollision
        % This is based upon the output of questions 2.5 and 2.6
        % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
        % and triangle obstacles in the environment (faces,vertex,faceNormals)
        function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
        
        for qIndex = 1:size(qMatrix,1)
            endEffectorToCenterDist = sqrt(sum((sphereCenter-tr(1:3,4)').^2));
            if endEffectorToCenterDist <= radius
                disp('Oh no a collision!');
        %         isCollision = 1;
            else
                disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
        %         isCollision = 0;
            end
                end    
            end
        end
    end

        %% GetLinkPoses
        % q - robot joint angles
        % robot -  seriallink robot model
        % transforms - list of transforms
        function [ transforms ] = GetLinkPoses( q, robot)
        
        links = robot.links;
        transforms = zeros(4, 4, length(links) + 1);
        transforms(:,:,1) = robot.base;
        
        for i = 1:length(links)
            L = links(1,i);
            
            current_transform = transforms(:,:, i);
            
            current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
            transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
            transforms(:,:,i + 1) = current_transform;
        end
        end
    end
end
  