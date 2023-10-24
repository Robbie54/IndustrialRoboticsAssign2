    classdef CollisionChecking < handle
    %% IsCollision
    % This is based upon the output of questions 2.5 and 2.6 in Lab 5
    % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
    % and triangle obstacles in the environment (faces,vertex,faceNormals)
    
    methods 
        function result = IsCollision(robot,qMatrix,returnOnceFound)
           
        if nargin < 3
            returnOnceFound = true;
        end
        result = false;
      
     
        centerpnt = [2,0,-0.5];
        side = 1.5;
        plotOptions.plotFaces = true;
        [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
        
        for qIndex = 1:size(qMatrix,1)
            % Get the transform of every joint (i.e. start and end of every link)
            tr = CollisionChecking.GetLinkPoses(qMatrix(qIndex,:), robot);
        
            % Go through each link and also each triangle face
            for i = 1 : size(tr,3)-1    
                for faceIndex = 1:size(faces,1)
                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                        plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                        display('Intersection');
                        result = true;
                        if returnOnceFound
                            return
                        end
                    end
                end    
            end
        end
        end
    end
    methods (Static)
        function [ transforms ] = GetLinkPoses( q, robot)
            
            links = robot.model.links;
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
        
        %% IsIntersectionPointInsideTriangle
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is 
        % inside (result == 1) or 
        % outside a triangle (result ==0 )
        function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)
        
        u = triangleVerts(2,:) - triangleVerts(1,:);
        v = triangleVerts(3,:) - triangleVerts(1,:);
        
        uu = dot(u,u);
        uv = dot(u,v);
        vv = dot(v,v);
        
        w = intersectP - triangleVerts(1,:);
        wu = dot(w,u);
        wv = dot(w,v);
        
        D = uv * uv - uu * vv;
        
        % Get and test parametric coords (s and t)
        s = (uv * wv - vv * wu) / D;
        if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
            result = 0;
            return;
        end
        
        t = (uv * wu - uu * wv) / D;
        if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
            result = 0;
            return;
        end
        
        result = 1;                      % intersectP is in Triangle
        end
    
    end
    end
    
    

