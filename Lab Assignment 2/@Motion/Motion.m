classdef Motion < handle

    properties
    end

    methods
        function self = Motion
            self.Functionality()
        end
    end

    methods (Static)
        function Functionality()

            %clear workspace, command window and figures
            clear all;
            close all;
            clf;


            %initialise robot and attach suction cup to the end effector
            r = Dobot()
            r.model;
            q = zeros(1,7);
            %q = zeros(1,7);
            b = SuctionCup();
            b.suctionModel{1}.base = r.model.fkine(r.model.getpos());
            pause;



            %define Matrices that hold the locations of the intial and
            %final Paper Stack

            %Define Number of Paper sheets to be manipulated
            paperNo = 9;

            %initial paper locations
            initialPaperMatrix = zeros(paperNo,3);
            %initialPaperMatrix(1,:) = ....


            %final paper locations
            finalPaperMatrix = zeros(paperNo,3);
            %finalPaperMatrix(1,:) = ....





            %create matrix that will store unique identifer to the paper so
            %they can be called in the future
            paperUniqueID = cell(paperNo, 1);

            % Create and store the paper identifier values via the loop.
            % spawn in brick at 0,0,0. translate by the initial brick
            % location matrix
            for paperIndex = 1:paperNo
                paperUniqueID{paperIndex} = PlaceObject('HalfSizedRedGreenBrick.ply'); %brickMatrix(brickIndex, :)
                vertices = get(paperUniqueID{paperIndex}, 'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(initialPaperMatrix(paperIndex,:))';
                set(paperUniqueID{paperIndex},'Vertices',transformedVertices(:,1:3));
            end

            %get initial starting robot position
            initialPosition = r.model.getpos();

            %Create a nested for loop to iterate through the robot arms
            %path and update the animation

            %For each brick 1-9:
            for paperIndex = 1:paperNo

                % Get the initial and final paper sheet location
                currentPaper = initialPaperMatrix(paperIndex, :);
                finalPaper = finalPaperMatrix(paperIndex, :);

                %set count variable to dictate how many joint angle paths
                %will be created to model the Path.
                count = 100;

                %get the robots current arm location
                robotLocation = r.model.getpos();

                %calculate the joint angles necessary ot traverse to the
                %chosen paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot grabs the paper from
                %the top.
                currentPaperPath = r.model.ikcon(transl(currentPaper)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and chosen brick location.
                currentQPath = jtraj(robotLocation, currentPaperPath, count);

                %for each joint step
                for i = 1:size(currentQPath, 1)
                    %animate the robots arm
                    r.model.animate(currentQPath(i, :));
                    drawnow();
                end



                % move to final Paper stack location

                %calculate the joint angles necessary to traverse to the
                %paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot places the paper from
                %the top.
                finalPaperPath = r.model.ikcon(transl(finalPaper)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and final paper location.
                finalQPath = jtraj(currentPaperPath, finalPaperPath, count);

                %for each joint step
                for i = 1:size(finalQPath, 1)

                    %animate arm
                    r.model.animate(finalQPath(i, :));

                    %calculate the end effector transform of the robot arm
                    endEffectorTransform = r.model.fkine(r.model.getpos()).T;

                    %update paper vertices to the new coordinates of the
                    %robots end effector
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';

                    %update the paper vertices with the transformed
                    %vertices
                    set(paperUniqueID{paperIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
                    drawnow();
                end
            end

            %move robot back to initial position
            count = 100;
            robotLocation = r.model.getpos();
            QPath = jtraj(robotLocation, initialPosition, count);
            for i = 1:size(QPath, 1)
                r.model.animate(QPath(i, :));
                drawnow();
            end
        end
    end
end