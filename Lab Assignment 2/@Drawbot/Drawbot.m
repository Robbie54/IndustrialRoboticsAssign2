classdef Drawbot < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)
        plyFileNameStem = 'Drawbot';
    end

    methods
        %% Constructor
        function self = Drawbot(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(-0.5,0,0.725);
                end
            else % All passed in
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end

            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            % q = zeros(4);
            % self.model.plot(q)

            drawnow
        end

        %% CreateModel
        function CreateModel(self)
            % link(1) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', deg2rad([-90,90]));
            link(1) = Link('d',0,'a',0.3,'alpha',0,'offset',-3*pi/8,'qlim', deg2rad([-90,90]));
            link(2) = Link('d',0,'a',0.3,'alpha',0,'offset',7*pi/8,'qlim', deg2rad([-180,180]));
            link(3) = Link('d',0,'a',0,'alpha',0,'offset',0);

            self.model = SerialLink(link,'name',self.name);

            
        end
    end
end
