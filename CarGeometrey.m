classdef CarGeometrey < handle
    properties
        CG=[];
        Theta=[];
        Width=[];
        Length=[];
        GraphicHandles=[];
    end
    methods
        function obj=CarGeometrey(CG,Theta,Width,Length)
        %just set paramereters
        obj.CG=CG;
        obj.Theta=Theta;
        obj.Width=Width;
        obj.Length=Length;
        end
        function DrawCar(obj,Ax)
        W=obj.Width;
        L=obj.Length;
        R=[cos(obj.Theta) -sin(obj.Theta);
            sin(obj.Theta) cos(obj.Theta)];
        purple=[0.5,0,0.5];
        yellow=[1,0.8,0];
        t=linspace(0,2*pi,7);
        t=t(1:end-1);
        r=W/6; %lights raidus
        
        %Draw carCG
        x_cg=[-L/2, L/2, L/2, -L/2];
        y_cg=[-W/2, -W/2, W/2, W/2];
        v_cg=[x_cg;y_cg];
        Mv_cg=R*v_cg+[obj.CG(1);obj.CG(2)];
        f_cg=[1,2,3,4];
        cgh=patch(Ax,'faces',f_cg,'vertices',Mv_cg','facecolor',purple);
        
        %Draw right light
        x_rl=r*cos(t)+L/2;
        y_rl=r*sin(t)+W/4;
        v_rl=[x_rl;y_rl];
        Mv_rl=R*v_rl+[obj.CG(1);obj.CG(2)];
        f_rl=[1,2,3,4,5,6];
        rlh=patch(Ax,'faces',f_rl,'vertices',Mv_rl','facecolor',yellow);
        
        %Draw left light
        x_ll=r*cos(t)+L/2;
        y_ll=r*sin(t)-W/4;
        v_ll=[x_ll;y_ll];
        Mv_ll=R*v_ll+[obj.CG(1);obj.CG(2)];
        f_ll=[1,2,3,4,5,6];
        llh=patch(Ax,'faces',f_ll,'vertices',Mv_ll','facecolor',yellow);
        
        obj.GraphicHandles=[cgh,rlh,llh];
        end
    end
end