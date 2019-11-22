classdef Kalman_Filter < handle  
    properties (SetAccess = private)
        crnt_state = carState() ; %x
        crnt_error  %P
    end%private properties
    properties %public
        
        
    end
    
    
    methods (Access = private)
        function [predicted_state, predicted_error ]  = predict_state(   crnt_state , sensors_data , delta_t) 
            %code here  % this is the dynamic model
        end
        function [crnt_state  , crnt_error    ]  = estimate_state( crnt_state,  predicted_state , predicted_error , sensors_data.measured_state  )
            %code here % here we update the 
        end
    end%private methods
    
    methods
        
        function obj = Kalman_Filter()
            obj.crnt_error = zeros( crnt_state.num_values ,  crnt_state.num_values  ) ;
        end
        
        function update(obj  , delta_t  , sensors_data)
            
            [predicted_state, predicted_error ]  =  predict_state(   obj.crnt_state , sensors_data , delta_t)   ;
            [obj.crnt_state  , obj.crnt_error    ]  =   estimate_state( obj.crnt_state ,  predicted_state , predicted_error , sensors_data.measured_state  ) ;
            
        end %update
        
        function state = get_currentState(obj)
            state = obj.crnt_state ;
        end
        
        
    end%methods
    
    
    
end%class_def