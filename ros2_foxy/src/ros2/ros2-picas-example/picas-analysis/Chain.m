classdef Chain
    properties
        id, type, num_callbacks, t_callback, r_callbacks, C, T, num_instance, ...
            trigger_next, next_r_callback_id, ...
            sem_priority
    end
    
    methods
        function obj = Chain(id, sem_prio)
            obj.id = id;
            obj.t_callback = [];
            obj.r_callbacks = [];
            obj.num_callbacks = 0;
            obj.C = 0;
            obj.T = 0;
            obj.type = 'p';
            obj.num_instance = 0;   % this is only for a non-periodic chain
            obj.trigger_next = false;
            obj.next_r_callback_id = 0;
            obj.sem_priority = sem_prio;
        end
        
        function obj = add_callback(obj, callback)
            if strcmp(callback.type, 'timer')
                obj.t_callback = [obj.t_callback callback];
                obj.T = callback.T;
            else
                obj.r_callbacks = [obj.r_callbacks callback];
            end
            obj.num_callbacks = length(obj.t_callback) + length(obj.r_callbacks);
            obj.C = obj.C + callback.C;
            
            for c = 1 : length(obj.t_callback)
                obj.t_callback(c).chain_c = obj.C;
            end
            for c = 1 : length(obj.r_callbacks)
                obj.r_callbacks(c).chain_c = obj.C;
            end
        end
    end
    
end

