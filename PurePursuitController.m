classdef PurePursuitController < handle
    properties
        Model=[]; %BicycleKinematicsModel/...
        L=[]; %Car Length
        WayPoints=[]; %mx2
        x=[];
        y=[];
        Theta=[];
        Kdd=[]; %ld=Kdd*v, ld=lookahead distance
    end
    methods
        function obj=PurePursuitController(Model,WayPoints,L,x0,y0,Theta0,Kdd)
            obj.Model=Model;
            obj.WayPoints=WayPoints;
            obj.L=L;
            obj.x=x0;
            obj.y=y0;
            obj.Theta=Theta0;
            obj.Kdd=Kdd;
        end
        function UpdatePose(obj,x,y,Theta)
           obj.x=x;
           obj.y=y;
           obj.Theta=Theta;
        end
        function [Delta,Q]=SuggestSteering(obj,V)
            %V=velocity of vehicle
            switch lower(obj.Model)
                case 'bicyclekinematicsmodel'
                    ld=obj.Kdd*V;
                    t=[cos(obj.Theta),sin(obj.Theta)];
                    P=[obj.x,obj.y];
                    Q=TargetPoint(P,t,ld,obj.WayPoints);
                    alpha=atan2(Q(2)-P(2),Q(1)-P(1))-obj.Theta;
                    Delta=atan2(2*obj.L*sin(alpha),ld);
            end
        end
    end
end

function Q=TargetPoint(P,t,ld,WayPoints)
FrontWayPoints=WayPoints((WayPoints-P)*t(:)>0,:);
SubR=vecnorm(FrontWayPoints-P,2,2)-ld;

FarPoints=FrontWayPoints(SubR>0,:);
[~,FarQInd]=min(vecnorm(FarPoints-P,2,2));
FarQ=FarPoints(FarQInd,:);
ClosePoints=FrontWayPoints(SubR<0,:);
[~,CloseQInd]=max(vecnorm(ClosePoints-P,2,2));
CloseQ=ClosePoints(CloseQInd,:);
Q=mean([FarQ;CloseQ]); %one could find the paramtereic intersection of circle and line but meh
end