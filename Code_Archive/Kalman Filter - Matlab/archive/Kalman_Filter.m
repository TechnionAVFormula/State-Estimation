 classdef Kalman_Filter < handle
    
    properties
        crntState = carState() ; %x
        crntError  ;%P
        
        n ;  % dimentionality of the system    /  number of values we estimate.
        lambda  = 3.5  ;  % Scaling factor for the uncented version....  Currently fake and ugly.
        
    end%private properties
  
    
    %{   Private methods:
        ...  those should stay hidden from who-ever is using our kalman fiter.
        ...  Everything that can make the main "Update"  method as simple as possible, should go here:
        ... }%

    methods (Access = private)
        
        function [predicted_state, predicted_error ]  = predict_state(   delta_t) 
       %%
       %{ Building the Sigma-Points matrix:  }%
       sigma_root = sqrtm(  obj.crntError  ); 
     
       positive_addition =  repmat( obj.crntState.vector , 1 , obj.n) + sqrt(obj.n + obj.lambda)*sigma_root ; 
       negativ_addition =  repmat( obj.crntState.vector , 1 , obj.n) - sqrt(obj.n + obj.lambda)*sigma_root ; 
       
       X = [  obj.crntState.vector , positive_addition  ,  negativ_addition ]  ; %Sigma-Point matrix
       
       %creating mu_'
       %creating sigma_'
       
            
        end
        
        
%         function [crnt_state  , crnt_error    ]  = estimate_state( crnt_state,  predicted_state , predicted_error , sensors_data.measured_state  )
%             %code here % here we update the 
%         end
    end%private methods
    
   
    %{ ================ Public Methods go here: ================ }%
    methods 
        
        % c'tor  for a kalman filter.
        function obj = Kalman_Filter()
            obj.crntState  = carState();
            obj.crntError = zeros( obj.crntState.numValues ,  obj.crntState.numValues  )  ;
            obj.n = obj.crntState.numValues;
        end
        
        
        %The actual prediction function that is needed to continously make predictions:
        % It can become very complicated and that's why we need to keep it a simple as possible.
        function update(obj  , delta_t  , sensors_data)
            
            [predicted_state, predicted_error ]  =  predict_state(    sensors_data , delta_t)   ;
           [obj.crnt_state  , obj.crnt_error    ]  =   estimate_state(  predicted_state , predicted_error , sensors_data.measured_state  ) ;

        end %update
        
        function state = get_currentState(obj)
            state = obj.crntState ;
        end
        
        
        %   Is this usefull ? I'm not sure...  
        function set_currentState(obj , currentCarState)
            obj.crnt_state = currentCarState;
        end
        
        
    end%methods
    
    
    
end%class_def

% %%
%     obj.crntError  =    [   [3, 7,  0 , 0] ; [ 0, 0.2  ,  0 , 0] ; [0.1, 0,  0.05 , 0]  ; [0, 0,  0 , 0.87]    ] 