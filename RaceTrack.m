classdef RaceTrack < handle
    properties 
        TrackWidth=5; %[m]
        MidNodes=[]; %mx2 [x,y]. Chosen by user
        MidIntrp=[] %nx2 [x,y]
        LeftCones=[]; %mx2 [x,y]
        RightCones=[]; %mx2 [x,y]
        ClosedLoop=0; %boolean
    end
    methods
        function AddNode(obj,P)
            %P is a new midpoint P=[x,y]
            obj.MidNodes(end+1,:)=P;
        end
        function DeleteNode(obj,N)
            obj.MidNodes(N,:)=[];
        end
        function LoopClose(obj)
            obj.MidNodes(end+1,:)=obj.MidNodes(1,:); %add first node as last node aswell
            obj.ClosedLoop=1; %changes the interpolation values
        end
        function PchipInterpolate(obj,N)
            m=size(obj.MidNodes,1);
            if N<m, error('interpolation points need to be bigger'), end
            
            time=linspace(0,1,N); %assume equally spaced nodes
            timenodes=linspace(0,1,m);
            Midpp=pchip(timenodes,obj.MidNodes');
            dMidpp=fnder(Midpp,1);
            
            v=ppval(dMidpp,time)'; 
            speed=vecnorm(v,2,2); %mx1 absolute of velocity of midnodes
            t=v./speed; %tangent vectors mx2
            n_left=[-t(:,2),t(:,1)];
            n_right=-n_left;
            
            obj.MidIntrp=ppval(Midpp,time)';
            obj.LeftCones=obj.MidIntrp+n_left*obj.TrackWidth/2;
            obj.RightCones=obj.MidIntrp+n_right*obj.TrackWidth/2;         
            
            if obj.ClosedLoop
                obj.LeftCones(end,:)=obj.LeftCones(1,:);
                obj.RightCones(end,:)=obj.RightCones(1,:);
            end
        end
        function LinearInterpolate(obj,N)
            m=size(obj.MidPoints,1);
            if N<m, error('interpolation points need to be bigger'), end
            
            time=linsapce(0,1,N); %assume equally spaced nodes
            timenodes=linspace(0,1,m);
            x=interp1(timenodes,obj.MidNodes(:,1),time,'linear');
            y=interp1(timenodes,obj.MidNodes(:,2),time,'linear');
            obj.MidIntrp=[x(:),y(:)];
            
            t=obj.MidIntrp(2:end,:)-obj.MidIntrp(1:end-1,:);
            t=t./normvec(t,2,2);
            t=[t(1,:);t];
            n_left=[-t(:,2),t(:,1)];
            n_right=-n_left;
            obj.LeftCones=obj.MidIntrp+n_left*obj.TrackWidth/2;
            obj.RightCones=obj.MidIntrp+n_right*obj.TrackWidth/2;
            
            if obj.ClosedLoop
                obj.LeftCones(end,:)=obj.LeftCones(1,:);
                obj.RightCones(end,:)=obj.RightCones(1,:);
            end
        end
        function handles=PlotSection_Linear(obj,Ax,varargin)
            if isempty(varargin),N=size(obj.MidNodes,1);  %if number of section not specified, draw last section
            else, N=varargin{1}; end
            
            %plots the section that connects midpoints N,N-1 linearly.
            %N=>1
            LeftColor=[0,0,1]; %blue
            RightColor=[0.9,0.9,0]; %yellow
            MidColor=[0,0,0]; %black
            ConeEdgeColor=[0.5,0.5,0];
            Msize=4;
            
            if N>2
                Mid=obj.MidNodes((N-2):N,:);
                t1=Mid(2,:)-Mid(1,:);
                t2=Mid(3,:)-Mid(2,:);
                t1=t1/norm(t1,2);
                t2=t2/norm(t2,2);
                
                n1_left=[-t1(2),t1(1)];
                n1_right=-n1_left;
                n2_left=[-t2(2),t2(1)];
                n2_right=-n2_left;
                
                Left(1,:)=Mid(2,:)+n1_left*obj.TrackWidth/2;
                Left(2,:)=Mid(3,:)+n2_left*obj.TrackWidth/2;
                Right(1,:)=Mid(2,:)+n1_right*obj.TrackWidth/2;
                Right(2,:)=Mid(3,:)+n2_right*obj.TrackWidth/2;
                               
                
                h1=plot(Ax,Left(:,1),Left(:,2),'color',LeftColor,...
                    'linewidth',2,'marker','o','linestyle','-','markersize',...
                    Msize,'markerfacecolor',LeftColor,'markeredgecolor',ConeEdgeColor);
                h2=plot(Ax,Right(:,1),Right(:,2),'color',RightColor,...
                    'linewidth',2,'marker','o','linestyle','-','markersize',...
                    Msize,'markerfacecolor',RightColor,'markeredgecolor',ConeEdgeColor);
                h3=plot(Ax,Mid(2:3,1),Mid(2:3,2),'color',MidColor,...
                    'linewidth',0.5,'linestyle','--');
                handles=[h1,h2,h3];   
            elseif N==2
                Mid=obj.MidNodes((N-1):N,:);
                t=Mid(2,:)-Mid(1,:);
                t=t/norm(t,2);
                
                n_left=[-t(2),t(1)];
                n_right=-n_left;
                
                Left(1,:)=Mid(1,:)+n_left*obj.TrackWidth/2;
                Left(2,:)=Mid(2,:)+n_left*obj.TrackWidth/2;
                Right(1,:)=Mid(1,:)+n_right*obj.TrackWidth/2;
                Right(2,:)=Mid(2,:)+n_right*obj.TrackWidth/2;
                
                
                h1=plot(Ax,Left(:,1),Left(:,2),'color',LeftColor,...
                    'linewidth',1,'marker','o','linestyle','-','markersize',...
                    Msize,'markerfacecolor',LeftColor,'markeredgecolor',ConeEdgeColor);
                h2=plot(Ax,Right(:,1),Right(:,2),'color',RightColor,...
                    'linewidth',1,'marker','o','linestyle','-','markersize',...
                    Msize,'markerfacecolor',RightColor,'markeredgecolor',ConeEdgeColor);
                h3=plot(Ax,Mid(:,1),Mid(:,2),'color',MidColor,...
                    'linewidth',0.5,'linestyle','--');
                handles=[h1,h2,h3];                
            
            elseif N==1
                handles=plot(Ax,obj.MidNodes(1),obj.MidNodes(2),'marker','o','markersize',Msize,...
                    'color',MidColor,'markeredgecolor',MidColor,'markerfacecolor',MidColor);
            end
        end
        function handles=PlotTrack(obj,Ax)
                LeftColor=[0,0,1]; %blue
                RightColor=[0.9,0.9,0]; %yellow
                MidColor=[0,0,0]; %black
                ConeEdgeColor=[0.5,0.5,0];
                Msize=4;
                
                h1=plot(Ax,obj.LeftCones(:,1),obj.LeftCones(:,2),'color',LeftColor,...
                    'linewidth',1,'marker','o','linestyle','-','markersize',...
                    Msize,'markerfacecolor',LeftColor,'markeredgecolor',ConeEdgeColor);
                h2=plot(Ax,obj.RightCones(:,1),obj.RightCones(:,2),'color',RightColor,...
                    'linewidth',1,'marker','o','linestyle','-','markersize',...
                    Msize,'markerfacecolor',RightColor,'markeredgecolor',ConeEdgeColor);
                h3=plot(Ax,obj.MidIntrp(:,1),obj.MidIntrp(:,2),'color',MidColor,...
                    'linewidth',0.5,'linestyle','--');
                handles=[h1,h2,h3];
        end
    end
end
    