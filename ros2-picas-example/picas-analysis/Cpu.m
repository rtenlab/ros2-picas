classdef Cpu
    properties
           id, utilization, hyperperiod, num_tasks, ...
           tasks_on_cpu, tasks, executors, executor_ids, ...
           ready_queue, on_core, ...
           low_sem_prio_chain, combined_executor_flag
    end
    
    methods
        function obj = Cpu(id)
            obj.id = id;
            obj.utilization = 0;
            obj.hyperperiod = 0;
            obj.num_tasks = 0;
            obj.executor_ids = [];
            obj.executors = [];
            obj.low_sem_prio_chain = 0;
            obj.combined_executor_flag = false;
        end
        % for analysis use
        function obj = assign_executor(obj, exe)
            exe.cpu = obj.id;
            for e = 1 : length(exe)
                for t = 1 : length(exe(e).callbacks)
                    exe(e).callbacks(t).cpu = obj.id;
                end
            end
            obj.executors = [obj.executors exe];
            obj.executor_ids = [obj.executor_ids exe.id];
            obj.utilization = obj.utilization + exe.util;
        end
        
        % for analysis use
        function obj = find_low_sem_prio_chain_cpu(obj, chains)
            tmp_sem_prio = inf;
            if ~isempty(obj.executors)
                for e = 1 : length(obj.executors)
                    tasks = obj.executors(e).callbacks;
                    for t = 1 : length(tasks)
                        if chains(tasks(t).chain_id).sem_priority < tmp_sem_prio
                            tmp_sem_prio = chains(tasks(t).chain_id).sem_priority;
                            obj.low_sem_prio_chain = tasks(t).chain_id;
                        end
                    end
                end
            end
        end
        
        function [obj chains] = assign_executors(obj, exe, chains)
            obj.executors = [obj.executors exe];
            obj.executor_ids = [obj.executor_ids exe.id];
            
            for i = 1 : length(exe)
                obj.num_tasks = obj.num_tasks + length(exe(i).callbacks);
            end
            
            % assign cpu to callbacks in chains
            for c = 1 : length(exe.callbacks)
                for i = 1 : length(chains)
                    if chains(i).t_callback.id == exe.callbacks(c).id
                        chains(i).t_callback.cpu = obj.id;
                    end
                    
                    for k = 1 : length(chains(i).r_callbacks)
                        if chains(i).r_callbacks(k).id == exe.callbacks(c).id
                            chains(i).r_callbacks(k).cpu = obj.id;
                        end
                    end
                end
            end
        end
        
        function [obj chains] = run_at_time(obj, t, chains)
            prev_core_t_id = 0;
            prev_core_idx = 0;
            prev_executor_id = 0;
            
            % Move a job on the core to ready_queue
            if ~isempty(obj.on_core)
                obj.ready_queue = [obj.ready_queue obj.on_core];
                prev_core_t_id = obj.on_core.task_id;
                prev_core_idx = obj.on_core.index;
                prev_executor_id = obj.on_core.executor_id;
                obj.on_core = [];
            end
            
            % distribute ready_queue jobs by executors
            for i = 1 : length(obj.ready_queue)
                exe_id = obj.ready_queue(i).executor_id;
                obj.executors(exe_id).ready_queue = [obj.executors(exe_id).ready_queue obj.ready_queue(i)];
            end
            
            % sort out the highest priority job from each executor
            sort_out_queue = [];
            for i = 1 : length(obj.executors)
                sort_out_queue = [sort_out_queue obj.executors(i).sort_ready_callbacks(prev_core_t_id, prev_core_idx)];
                obj.executors(i).ready_queue = [];
            end
            
            if ~isempty(sort_out_queue)
                [~, idx] = sort([sort_out_queue.priority], 'descend');
                sort_out_queue = sort_out_queue(idx);
                
                % Put the highest priority job on the core
                obj.on_core = sort_out_queue(1);
                % remove from ready_queue
                for r = 1 : length(obj.ready_queue)
                    if obj.ready_queue(r).task_id == obj.on_core.task_id && obj.ready_queue(r).index == obj.on_core.index
                        obj.ready_queue(r) = [];
                        break;
                    end
                end                
%                 obj.sort_out_queue(1) = [];

                % Trace in & out for jobs
                if obj.on_core.task_id ~= prev_core_t_id || obj.on_core.index ~= prev_core_idx      % Changed and -> or
                    obj.on_core.trace_in = [obj.on_core.trace_in t];
                    for i = 1 : length(obj.ready_queue)
                        if obj.ready_queue(i).task_id == prev_core_t_id && obj.ready_queue(i).index == prev_core_idx
                            obj.ready_queue(i).trace_out = [obj.ready_queue(i).trace_out t];
                        end
                    end
                end
            end
            
            
            if ~isempty(obj.on_core)
                [obj.on_core chains] = obj.on_core.execute_time_unit(t, chains);
                
                if obj.on_core.complete_flag
                    obj.on_core.trace_out = [obj.on_core.trace_out t+1];
                    
                    for c = 1 : length(chains)
                        if ~isempty(obj.on_core)
                            for b = 1 : length(chains(c).t_callback)
                                if chains(c).t_callback(b).id == obj.on_core.task_id
                                    chains(c).t_callback(b).jobs = [chains(c).t_callback(b).jobs obj.on_core];
                                    obj.on_core = [];
                                    break;
                                end
                            end

                            if ~isempty(obj.on_core)
                                for b = 1 : length(chains(c).r_callbacks)
                                    if chains(c).r_callbacks(b).id == obj.on_core.task_id
                                        chains(c).r_callbacks(b).jobs = [chains(c).r_callbacks(b).jobs obj.on_core];
                                        obj.on_core = [];
                                        break;
                                    end
                                end 
                            end
                        end
                    end                    
                end
            end
            
            % Check deadline missed job and drop it
%             del_idx = [];
%             for i = 1 : length(obj.ready_queue)
%                 if obj.ready_queue(i).current_exe < obj.ready_queue(i).wcet && obj.ready_queue(i).abs_D == t
%                     obj.ready_queue(i).meet = false;
%                     obj.ready_queue(i).miss_deadline = true;
%                     obj.ready_queue(i).complete_flag = false;
%                     obj.ready_queue(i).complete_t = inf;
%                     
%                     if prev_core_t_id == obj.ready_queue(i).task_id && prev_core_idx == obj.ready_queue(i).index
%                         obj.ready_queue(i).trace_out = [obj.ready_queue(i).trace_out t];
%                     end
%                     
%                     for t_id = 1 : length(obj.tasks)
%                         if obj.tasks(t_id).id == obj.ready_queue(i).task_id
%                             obj.tasks(t_id).jobs = [obj.tasks(t_id).jobs obj.ready_queue(i)];
% %                             obj.on_core = [];
%                             break;
%                         end
%                     end
%                     del_idx = [del_idx i];
%                 end
%             end
%             obj.ready_queue(del_idx) = [];
                        
            
            % Sort jobs on ready_queue by descending order of priorities &
            % executors
%             if ~isempty(obj.ready_queue)
%                 [~, idx] = sort([obj.ready_queue.index], 'ascend');
%                 obj.ready_queue = obj.ready_queue(idx);
%                 
%                 [~, idx] = sort([obj.ready_queue.priority], 'descend');
%                 obj.ready_queue = obj.ready_queue(idx);
%                 
%                 % Put the highest priority job on the core
%                 obj.on_core = obj.ready_queue(1);
%                 obj.ready_queue(1) = [];
% 
%                 % Trace in & out for jobs
%                 if obj.on_core.task_id ~= prev_core_t_id || obj.on_core.index ~= prev_core_idx      % Changed and -> or
%                     obj.on_core.trace_in = [obj.on_core.trace_in t];
%                     for i = 1 : length(obj.ready_queue)
%                         if obj.ready_queue(i).task_id == prev_core_t_id && obj.ready_queue(i).index == prev_core_idx
%                             obj.ready_queue(i).trace_out = [obj.ready_queue(i).trace_out t];
%                         end
%                     end
%                 end
%             end
            
%             if ~isempty(obj.on_core)
%                 obj.on_core = obj.on_core.execute_time_unit(t);
%                 
%                 if obj.on_core.complete_flag
%                     obj.on_core.trace_out = [obj.on_core.trace_out t+1];
%                     for i = 1 : length(obj.tasks)
%                         if obj.tasks(i).id == obj.on_core.task_id
%                             obj.tasks(i).jobs = [obj.tasks(i).jobs obj.on_core];
%                             obj.on_core = [];
%                             break;
%                         end
%                     end
%                 end
%             end
            
        end
    end
end

