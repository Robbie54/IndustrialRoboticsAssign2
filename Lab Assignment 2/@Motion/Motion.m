classdef Motion < handle
    methods
        function self = Motion
        end
    end
    methods (Static)
        function Functionality()
            r = Dobot()
            r.model;
            q = zeros(1,7);
            b = SuctionCup();
            b.suctionModel{1}.base = r.model.fkine(r.model.getpos());
        end
    end
end
