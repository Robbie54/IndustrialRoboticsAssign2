classdef Motion < handle

    properties (Constant)
        paperNo = 5;
        %ColourPoints = 10;
        initialPaper = Motion.initialPaperMatrix();
        finalPaper = Motion.finalPaperMatrix();
        boardLocation = Motion.drawingBoardMatrix();
    end
    properties
        % r1 = UR3();
        % r2 = Drawbot();
    end

    methods
        function self = Motion
            self.RobotMotion()
        end
    end

    methods (Static)
        function RobotMotion()
            %clear workspace, command window and figures
            clear all;
            close all;
            clf;

            %initialise robot and attach suction cup to the end effector
            hold on;
            r1 = UR3();
            r2 = Drawbot();
            r1.model;
            r2.model;
            %q = zeros(1,7);
            % b = SuctionCup();
            % b.suctionModel{1}.base = r1.model.fkine(r1.model.getpos());
            %pause;

            axis equal;

            %initialise constant properties into function
            paperNo = Motion.paperNo;
            %ColourPoints = Motion.ColourPoints;
            initialPaper = Motion.initialPaper;
            finalPaper = Motion.finalPaper;
            boardLocation = Motion.boardLocation;
            baseTr = [0.2, -0.4, 0.7];

            %create matrix that will store unique identifer to the paper so
            %they can be called in the future
            paperUniqueID = cell(paperNo, 1);

            % Create and store the paper identifier values via the loop.
            % spawn in brick at 0,0,0. translate by the initial brick
            % location matrix
            for paperIndex = 1:paperNo
                paperUniqueID{paperIndex} = PlaceObject('papersheet_industrial_flipped.ply');%, [0.2, -0.4, 0.7]); %brickMatrix(brickIndex, :)
                vertices = get(paperUniqueID{paperIndex}, 'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(initialPaper(paperIndex,:))'; %* transl(baseTr); %transl(initialPaper(paperIndex,:))';
                set(paperUniqueID{paperIndex},'Vertices',transformedVertices(:,1:3));
            end

            %Create a nested for loop to iterate through the robot arms
            %path and update the animation

            %For each brick 1-9:
            for paperIndex = 1:paperNo

                %finding matrix positions necessary
                initialLocation = initialPaper(paperIndex,:);
                middleLocation = boardLocation;
                finalLocation = finalPaper(paperIndex,:);

                %moving paper from initial stack to drawing board
                placePaperToDrawingBoard(paperIndex, initialLocation, middleLocation);
                

                %draw on page
                count = 20;
                ColourPoints = 9;
                dist = -0.04;

                %initial paper locations
                Drawing = zeros(ColourPoints,3);
                %initialPaperMatrix(1,:) = ....
                Drawing(1,:) = [0.01, 0.07, 0.75];
                Drawing(2,:) = [0.01, -0.07, 0.75];
                Drawing(3,:) = [-0.04, -0.08, 0.75];
                Drawing(4,:) = [-0.06, -0.06, 0.75];
                Drawing(5,:) = [-0.08, -0.03, 0.75];
                Drawing(6,:) = [-0.09, 0, 0.75];
                Drawing(7,:) = [-0.08, 0.03, 0.75];
                Drawing(8,:) = [-0.06, 0.06, 0.75];
                Drawing(9,:) = [-0.04, 0.08, 0.75];
                Drawing(10,:) = [-0.04, 0, 0.75];

                rsp = r2.model.getpos;
                r2p1 = r2.model.ikcon(transl(Drawing(1,:)));
                r2p2 = r2.model.ikcon(transl(Drawing(2,:)));
                r2p3 = r2.model.ikcon(transl(Drawing(3,:)));
                r2p4 = r2.model.ikcon(transl(Drawing(4,:)));
                r2p5 = r2.model.ikcon(transl(Drawing(5,:)));
                r2p6 = r2.model.ikcon(transl(Drawing(6,:)));
                r2p7 = r2.model.ikcon(transl(Drawing(7,:)));
                r2p8 = r2.model.ikcon(transl(Drawing(8,:)));
                r2p9 = r2.model.ikcon(transl(Drawing(9,:)));
                r2p10 = r2.model.ikcon(transl(Drawing(10,:)));


                %% Pathing and Animation
                % Pathing from Start to 1st Position
                %below is for each position of the drawbot to draw 
                qPath1 = jtraj(rsp,r2p1,count);

                %check for collision in 1st motion can be done for each
                %collision object is in the isCollision method 
                Motion.IsCollision(r2,qPath1)

                for i = 1:length(qPath1)
                    r2.model.animate(qPath1(i,:));
                    drawnow();
                end

                tr = r2.model.fkine(r2.model.getpos).T;
                endP = tr(1:3,4)' + dist * tr(1:3,3)';
                plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                pause(0.1);
                
                qPath2 = jtraj(r2p1,r2p2,count);
                for i = 1:length(qPath2)
                    r2.model.animate(qPath2(i,:));
                    drawnow();
                end
                
                tr = r2.model.fkine(r2.model.getpos).T;
                endP = tr(1:3,4)' + dist * tr(1:3,3)';
                plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                pause(0.1);
                
                qPath3 = jtraj(r2p2,r2p3,count);
                for i = 1:length(qPath3)
                    r2.model.animate(qPath3(i,:));
                    drawnow();
                end

                tr = r2.model.fkine(r2.model.getpos).T;
                endP = tr(1:3,4)' + dist * tr(1:3,3)';
                plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                pause(0.1);

                qPath4 = jtraj(r2p3,r2p4,count);
                for i = 1:length(qPath4)
                    r2.model.animate(qPath4(i,:));
                    drawnow();
                    tr = r2.model.fkine(r2.model.getpos).T;
                    endP = tr(1:3,4)' + dist * tr(1:3,3)';
                    plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                    pause(0.1);
                end

                qPath5 = jtraj(r2p4,r2p5,count);
                for i = 1:length(qPath5)
                    r2.model.animate(qPath5(i,:));
                    drawnow();
                    tr = r2.model.fkine(r2.model.getpos).T;
                    endP = tr(1:3,4)' + dist * tr(1:3,3)';
                    plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                    pause(0.1);
                end

                qPath6 = jtraj(r2p5,r2p6,count);
                for i = 1:length(qPath6)
                    r2.model.animate(qPath6(i,:));
                    drawnow();
                    tr = r2.model.fkine(r2.model.getpos).T;
                    endP = tr(1:3,4)' + dist * tr(1:3,3)';
                    plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                    pause(0.1);
                end

                qPath7 = jtraj(r2p6,r2p7,count);
                for i = 1:length(qPath7)
                    r2.model.animate(qPath7(i,:));
                    drawnow();
                    tr = r2.model.fkine(r2.model.getpos).T;
                    endP = tr(1:3,4)' + dist * tr(1:3,3)';
                    plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                    pause(0.1);
                end

                qPath8 = jtraj(r2p7,r2p8,count);
                for i = 1:length(qPath8)
                    r2.model.animate(qPath8(i,:));
                    drawnow();
                    tr = r2.model.fkine(r2.model.getpos).T;
                    endP = tr(1:3,4)' + dist * tr(1:3,3)';
                    plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                    pause(0.1);
                end

                qPath9 = jtraj(r2p8,r2p9,count);
                for i = 1:length(qPath9)
                    r2.model.animate(qPath9(i,:));
                    drawnow();
                    tr = r2.model.fkine(r2.model.getpos).T;
                    endP = tr(1:3,4)' + dist * tr(1:3,3)';
                    plot3(endP(1) + 0.04,endP(2),endP(3),'y*');
                    pause(0.1);
                end

                qPath10 = jtraj(r2p9,r2p10,count);
                for i = 1:length(qPath10)
                    r2.model.animate(qPath10(i,:));
                    drawnow();
                end

                qPath11 = jtraj(r2p10,rsp,count);
                for i = 1:length(qPath11)
                    r2.model.animate(qPath11(i,:));
                    drawnow();
                end
                

                %moving paper from drawing board to final stack
                placePaperToStack(paperIndex, middleLocation, finalLocation);

            end

            function placePaperToDrawingBoard(paperIndex,initialLocation,finalLocation)


                %set count variable to dictate how many joint angle paths
                %will be created to model the Path.
                count = 100;

                %get the robots current arm location
                robotLocation = r1.model.getpos();

                %calculate the joint angles necessary to traverse to the
                %chosen paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot grabs the paper from
                %the top.

                %updatedLocation = transl(initialLocation)*troty(pi);

                currentPaperPath = r1.model.ikcon(transl(initialLocation)*troty(pi));
                %currentPaperPath = r1.model.ikine(updatedLocation, 'q0', newQ, 'mask', [1,1,0,0,0,0]);

                %computes a joint space trajectory that inrerpolates
                %between the robots position and chosen brick location.
                currentQPath = jtraj(robotLocation, currentPaperPath, count);

                %for each joint step
                for j = 1:size(currentQPath, 1)
                    %animate the robots arm
                    r1.model.animate(currentQPath(j, :));
                    drawnow();
                end

                %% move to final Paper stack location
                %calculate the joint angles necessary to traverse to the
                %paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot places the paper from
                %the top.
                finalPaperPath = r1.model.ikcon(transl(finalLocation)*troty(pi));

                q = currentPaperPath;
                testPaperPath = r1.model.ikine(transl(finalLocation)*troty(pi), 'q0', q, 'mask', [0,0,0,0,1,1]);

                %computes a joint space trajectory that inrerpolates
                %between the robots position and final paper location.
                currentPaperPath;
                finalQPath = jtraj(testPaperPath, finalPaperPath, count);

                %for each joint step
                for h = 1:size(finalQPath, 1)

                    %animate arm
                    r1.model.animate(finalQPath(h, :));

                    %calculate the end effector transform of the robot arm
                    endEffectorTransform = r1.model.fkine(r1.model.getpos()).T;

                    %update paper vertices to the new coordinates of the
                    %robots end effector
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';

                    %update the paper vertices with the transformed
                    %vertices
                    set(paperUniqueID{paperIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
                    drawnow();
                end

                %%move robot back to initial position
                count = 100;
                q = [pi/2 3*pi/2 0 0 0 0];
                robotLocation = r1.model.getpos();
                QPath = jtraj(robotLocation, q, count);
                for h = 1:size(QPath, 1)
                    r1.model.animate(QPath(h, :));
                    drawnow();
                end
            end

            function placePaperToStack(paperIndex,initialLocation,finalLocation)
                %set count variable to dictate how many joint angle paths
                %will be created to model the Path.
                count = 100;

                %get the robots current arm location
                robotLocation = r1.model.getpos();

                %calculate the joint angles necessary to traverse to the
                %chosen paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot grabs the paper from
                %the top.
                currentPaperPath = r1.model.ikcon(transl(initialLocation)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and chosen brick location.
                currentQPath = jtraj(robotLocation, currentPaperPath, count);

                %for each joint step
                for j = 1:size(currentQPath, 1)
                    %animate the robots arm
                    r1.model.animate(currentQPath(j, :));
                    drawnow();
                end

                %% move to final Paper stack location
                %calculate the joint angles necessary to traverse to the
                %paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot places the paper from
                %the top.
                finalPaperPath = r1.model.ikcon(transl(finalLocation)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and final paper location.
                finalQPath = jtraj(currentPaperPath, finalPaperPath, count);

                %for each joint step
                for h = 1:size(finalQPath, 1)

                    %animate arm
                    r1.model.animate(finalQPath(h, :));

                    %calculate the end effector transform of the robot arm
                    endEffectorTransform = r1.model.fkine(r1.model.getpos()).T;

                    %update paper vertices to the new coordinates of the
                    %robots end effector
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';

                    %update the paper vertices with the transformed
                    %vertices
                    set(paperUniqueID{paperIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
                    drawnow();
                end
            end

        end

        function initialPaper = initialPaperMatrix()
            %initial paper stack
            paperNo = Motion.paperNo;

            initialPaperMatrix = zeros(paperNo,3);
            %initialPaperMatrix(1,:) = ....
            initialPaperMatrix(1,:) = [0.2, -0.4, 0.7];
            initialPaperMatrix(2,:) = [0.2, -0.4, 0.71];
            initialPaperMatrix(3,:) = [0.2, -0.4, 0.72];
            initialPaperMatrix(4,:) = [0.2, -0.4, 0.73];
            initialPaperMatrix(5,:) = [0.2, -0.4, 0.74];
            initialPaper = initialPaperMatrix;
        end

        function finalPaper = finalPaperMatrix()
            %final paper stack
            paperNo = Motion.paperNo;

            finalPaperMatrix = zeros(paperNo,3);
            %finalPaperMatrix(1,:) = ....
            finalPaperMatrix(1,:) = [0.2, 0.4, 0.7];
            finalPaperMatrix(2,:) = [0.2, 0.4, 0.71];
            finalPaperMatrix(3,:) = [0.2, 0.4, 0.72];
            finalPaperMatrix(4,:) = [0.2, 0.4, 0.73];
            finalPaperMatrix(5,:) = [0.2, 0.4, 0.74];
            finalPaper = finalPaperMatrix;
        end

        function boardLocation = drawingBoardMatrix()
            %drawing board
            drawingBoardMatrix = zeros(1,3);
            drawingBoardMatrix(1,:) = [0,0,0.7];
            boardLocation = drawingBoardMatrix;
        end

        function result = IsCollision(robot,qMatrix,returnOnceFound)
           
            if nargin < 3
                returnOnceFound = true;
            end
            result = false;
          
         
            centerpnt = [0,0,0.7];
            side = 0.1;
            plotOptions.plotFaces = true;
            [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
            
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = Motion.GetLinkPoses(qMatrix(qIndex,:), robot);
            
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1    
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                        if check == 1 && Motion.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                            display('Collision iminent abort!');
                            result = true;
                            if returnOnceFound
                                return
                            end
                        end
                    end    
                end
            end
        end
        
    
        function [ transforms ] = GetLinkPoses( q, robot)
            links = robot.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.model.base;
            
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
