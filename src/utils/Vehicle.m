classdef Vehicle
    % VEHICLE
    %   Class which contains all relevant vehicle model parameters.
    
    properties
        m    % mass
        Jz   % moment of inertia around z axis
        lf   % distance between the centre of gravity and the front axle
        lr   % distance between the centre of gravity and the rear axle
        w    % half of the vehicles width
        cw   % air drag coefficient
        Aw   % projected area in a transversal view
        frontAxle % front axle data
        rearAxle % rear axle data
        steerRatio % steering wheel ratio
        Ku % understeering gradient
    end
    
    methods
        function obj = Vehicle(m, Jz, lf, lr, w, cw, Aw, frontAxle, rearAxle, steerRatio, Ku)
           % initialize vehicle object
           obj.m = m;
           obj.Jz = Jz;
           obj.lf = lf;
           obj.lr = lr;
           obj.w = w;
           obj.cw = cw;
           obj.Aw = Aw; 
           obj.frontAxle = frontAxle;
           obj.rearAxle = rearAxle;
           obj.steerRatio = steerRatio;
           obj.Ku = Ku;
        end
        
        function paramList = ReturnParams(obj)
             
           paramList = {obj.lf; 
                        obj.lr; 
                        obj.w;
                        obj.m;
                        obj.Jz;
                        obj.cw;
                        obj.Aw;
                        obj.steerRatio;
                        obj.Ku;
                        [obj.frontAxle.Rw, obj.rearAxle.Rw];
                        [obj.frontAxle.Jw, obj.rearAxle.Jw];
                        [obj.frontAxle.Cx, obj.rearAxle.Cx];
                        [obj.frontAxle.Cy, obj.rearAxle.Cy];
                        [obj.frontAxle.rollRes, obj.rearAxle.rollRes]};
        end
        
    end
end

