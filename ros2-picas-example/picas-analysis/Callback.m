classdef Callback
    
    properties
        id, type, T, C, D, executor, priority, cpu, ...
            chain_id, chain_order, chain_T, ...
            chain_c, ...
            wcrt, ...
            job_index, jobs,
            segment_flag, segment_C, chain_on_cpu     % for analysis purpose
    end
    
    methods
        function obj = Callback(id, period, execution, chain_id, order, cpu_id, executor_id)
            obj.id = id;
            obj.T = period;
            if period ~= 0
                obj.type = 'timer';
                obj.D = period;
            else
                obj.type = 'regular';
            end
            obj.C = execution;
            obj.executor = 0;
            obj.priority = 0;
            obj.chain_id = chain_id;
            obj.chain_order = order;
            obj.chain_c = 0;
            obj.wcrt = 0;
            obj.job_index = 1;
            obj.jobs = [];
            obj.segment_flag = false;
            obj.segment_C = 0;
            obj.chain_on_cpu = false;
            if nargin > 5
                obj.executor = executor_id;
                obj.cpu = cpu_id;
            end
        end
    end
end

