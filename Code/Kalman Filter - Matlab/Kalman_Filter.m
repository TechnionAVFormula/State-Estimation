classdef Kalman_Filter < handle
    properties (SetAccess = private)
        crnt_state = carState() ; %x
        crnt_error  %P
    end%private properties
    properties %public
        
        
    end
    
    %{   Private methods:
        ...  those should stay hidden from who-ever is using our kalman fiter.
        ...  Everything that can make the main "Update"  method as simple as possible, should go here:
        ... }%

    methods (Access = private)
        
%         function [predicted_state, predicted_error ]  = predict_state(   crnt_state , sensors_data , delta_t) 
%             %code here  % this is the dynamic model
%         end
%         function [crnt_state  , crnt_error    ]  = estimate_state( crnt_state,  predicted_state , predicted_error , sensors_data.measured_state  )
%             %code here % here we update the 
%         end
    end%private methods
    
   
    %{ ================ Public Methods go here: ================ }%
    methods 
        
        % c'tor  for a kalman filter.
        function obj = Kalman_Filter()
            obj.crnt_state  = carState();
            obj.crnt_error = zeros( obj.crnt_state.numValues ,  obj.crnt_state.numValues  )  ;
        end
        
        
        %The actual prediction function that is needed to continously make predictions:
        % It can become very complicated and that's why we need to keep it a simple as possible.
        function update(obj  , delta_t  , sensors_data)
            
            [predicted_state, predicted_error ]  =  predict_state(   obj.crnt_state , sensors_data , delta_t)   ;
            [obj.crnt_state  , obj.crnt_error    ]  =   estimate_state( obj.crnt_state ,  predicted_state , predicted_error , sensors_data.measured_state  ) ;

        end %update
        
        function state = get_currentState(obj)
            state = obj.crnt_state ;
        end
        
        
        %   Is this usefull ? I'm not sure...  
        function set_currentState(obj , currentCarState)
            obj.crnt_state = currentCarState;
        end
        
        
    end%methods
    
    
    
end%class_def