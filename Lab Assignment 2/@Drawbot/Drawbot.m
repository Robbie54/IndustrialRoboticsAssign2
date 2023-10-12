classdef Drawbot < handle
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Property1
    end

    methods
        function self = Drawbot
            self.Run()
        end
    end

    methods (Static)
        %% CreateModel
        function Run()
            %refer to notes on dobot arm pdfs
            link(1) = Link('d',0.2,'a',0,'alpha',0,'offset',0,'qlim', deg2rad([-135,135]));
            link(2) = Link('d',0,'a',1,'alpha',0,'offset',-3*pi/8,'qlim', deg2rad([-5, 85]));
            link(3) = Link('d',0,'a',1,'alpha',0,'offset',7*pi/8,'qlim', deg2rad([-10,95]));
            link(4) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', deg2rad([-90,90]));
            % link(5) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', deg2rad([-85,85]));

            robot = SerialLink([link(1) link(2) link(3) link(4)],'name','Drawbot');                     % Generate the model

            workspace = [-3 3 -3 3 -0 1];                                       % Set the size of the workspace when drawing the robot
            scale = 0.5;
            q = zeros(1,4);                                                     % Create a vector of initial joint angles
            robot.plot(q,'workspace',workspace,'scale',scale);
        end
    end
end
