classdef BicycleKinematicModel <handle
    %RearAxle model!
    
    %Theta - tan(x,y)
    %Delta - steering angle with respect to Theta
    %Psi - rate of change of steering angle
    %V - velocity
    properties
        WheelBase=1 %distance between front and rear axles
        MaxSteeringAngle=pi/4;
    end
    properties (SetAccess=private)
        State=[0,0,0,0]'; %[x,y,Theta,Delta]
    end
    methods
        function obj=BicycleKinematicModel(L,x0,y0,Theta,Delta) %class constructor
            obj.WheelBase=L;
            obj.State=[x0,y0,Theta,Delta]';
        end
        function ProgressModel(obj,T,V,Psi)
            tspan=[0,T];
            if abs(obj.State(4))+abs(Psi*T)>obj.MaxSteeringAngle
                Psi=obj.State(4)/T;
            end
            [time,timeStates]=ode45(@(t,z) odefun(t,z,V,Psi,obj.WheelBase),tspan,obj.State);
            obj.State=timeStates(end,:)';
        end
    end      
end
function dzdt=odefun(t,z,V,Psi,L)
%z - State [x,y,theta,delta]
%V - velocity
%Psi - ddelta/dt
%L - wheelbase
dzdt=zeros(4,1);
dzdt(1)=V*cos(z(3));
dzdt(2)=V*sin(z(3));
dzdt(3)=V*tan(z(4))/L;
dzdt(4)=Psi;
end
%{ 
%Testing
L=1;
x0=0;
y0=0;
Theta=0;
Delta=pi/4;
Car=BicycleKinematicModel(L,x0,y0,Theta,Delta);

V=1;
Psi=0;

T=2;
N=100;
dt=T/N;
Rear=zeros(N,2);
for kk=1:N
Car.ProgressModel(dt,V,Psi);
Rear(kk,:)=Car.State([1,2])';
end

plot(Rear(:,1),Rear(:,2));
axis('equal') 
%}