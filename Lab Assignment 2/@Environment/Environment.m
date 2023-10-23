%% Class Creation
classdef Environment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here

    %% Method Setup
    methods
        function self = Environment()
            self.Run();
        end
    end

    methods (Static)
        %% Creates Primay Function
        function Run()
            % Updating User iwth Program Progress
            disp('Generating Environment');
                
            % Allowing all Models to be Placed Over Each Other
            hold on;
            
            % Sets the View and Workspace for the Environment
            view([3 -2 1.5]);
            axis([-2.5, 2.5, -2.5, 2.5, 0, 3])
            
            % Covers the Floor and Walls in Concrete
            surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg') ,'FaceColor','texturemap');
            surf([-2.5,-2.5;-2.5,-2.5],[-2.5,2.5;-2.5,2.5], [3,3;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-2.5,-2.5;2.5,2.5],[2.5,2.5;2.5,2.5], [3,0;3,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-2.5,-2.5;-2.5,-2.5], [0.5,0.5;-0.5,-0.5], [1,2;1,2],'CData',imread('WarningSignR.jpg'),'FaceColor','texturemap');
            
            % Places the Tables, the Estop Button, the Fire Extinguisher
            % and the Fences
            PlaceObject('table.ply',[0,0.5,0.6]);
            PlaceObject('table.ply',[0,-0.5,0.6]);
            % PlaceObject('tableBlue1x1x0.5m.ply',[0.6,-0.5,0]);
            % PlaceObject('tableBlue1x1x0.5m.ply',[-0.6,-0.5,0]);
            h = PlaceObject('emergencyStopWallMounted.ply', [0,-2.5,-0.7]);
            verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotx(pi);
            set(h,'Vertices',verts(:,1:3))
            PlaceObject('fireExtinguisher.ply', [-2.3,2.3,0]);
            f1 = PlaceObject('barrier1.5x0.2x1m.ply', [0.75,-1.5,0]);
            verts = [get(f1,'Vertices'), ones(size(get(f1,'Vertices'),1),1)] * trotz(0);
            set(f1,'Vertices',verts(:,1:3))
            f2 = PlaceObject('barrier1.5x0.2x1m.ply', [-0.75,-1.5,0]);
            verts = [get(f2,'Vertices'), ones(size(get(f2,'Vertices'),1),1)] * trotz(pi/2);
            set(f2,'Vertices',verts(:,1:3))
            f3 = PlaceObject('barrier1.5x0.2x1m.ply', [-0.75,-1.5,0]);
            verts = [get(f3,'Vertices'), ones(size(get(f3,'Vertices'),1),1)] * trotz(pi);
            set(f3,'Vertices',verts(:,1:3))
            f4 = PlaceObject('barrier1.5x0.2x1m.ply', [0.75,-1.5,0]);
            verts = [get(f4,'Vertices'), ones(size(get(f4,'Vertices'),1),1)] * trotz(3*pi/2);
            set(f4,'Vertices',verts(:,1:3))
            f5 = PlaceObject('barrier1.5x0.2x1m.ply', [0.75,-1.5,0]);
            verts = [get(f5,'Vertices'), ones(size(get(f5,'Vertices'),1),1)] * trotz(pi/2);
            set(f5,'Vertices',verts(:,1:3))
            f6 = PlaceObject('barrier1.5x0.2x1m.ply', [-0.75,-1.5,0]);
            verts = [get(f6,'Vertices'), ones(size(get(f6,'Vertices'),1),1)] * trotz(3*pi/2);
            set(f6,'Vertices',verts(:,1:3))
            f7 = PlaceObject('barrier1.5x0.2x1m.ply', [0.75,-1.5,0]);
            verts = [get(f7,'Vertices'), ones(size(get(f7,'Vertices'),1),1)] * trotz(pi);
            set(f7,'Vertices',verts(:,1:3))
            f8 = PlaceObject('barrier1.5x0.2x1m.ply', [-0.75,-1.5,0]);
            verts = [get(f8,'Vertices'), ones(size(get(f8,'Vertices'),1),1)] * trotz(0);
            set(f8,'Vertices',verts(:,1:3))

            % Updating User of Current Progress
            disp('Environment Generation Complete');
        end
    end
end

