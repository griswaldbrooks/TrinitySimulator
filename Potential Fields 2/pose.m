classdef pose
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x = 0;
        y = 0;
    end
    
    methods
        function obj = pose(x, y)
            obj.x = x;
            obj.y = y;
        end
        function [r, th] = polar(obj)
            r = sqrt(obj.x^2 + obj.y^2);
            th = atan2(obj.y,obj.x);
        end
    end
    
end

