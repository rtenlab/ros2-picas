function [chains chain_latency] = response_time_callbacks(chains, cpus)

    % reshape callbacks and distinguish segment tasks for each chain/CPU
    callbacks = [];
    executors = [];
    
    idx = 1;
    for c = 1 : length(cpus)        
        chain_segment_priority = inf*ones(1, length(chains));
        chain_segment_task_idx = zeros(1, length(chains));
        chain_segment_exe_time = zeros(1, length(chains));
        for e = 1 : length(cpus(c).executors)        
            for t = 1 : length(cpus(c).executors(e).callbacks)
                cur_cb = cpus(c).executors(e).callbacks(t);
                cur_chain = cur_cb.chain_id;
                if cur_cb.priority < chain_segment_priority(cur_chain)
                    chain_segment_priority(cur_chain) = cur_cb.priority;
                    chain_segment_task_idx(cur_chain) = idx;
                end
                chain_segment_exe_time(cur_chain) = chain_segment_exe_time(cur_chain) + cur_cb.C;
                callbacks = [callbacks cur_cb];
                idx = idx + 1;
            end
            executors = [executors cpus(c).executors(e)];
        end
        
        % set segment callbacks
        for s = 1 : length(chains)
            % remove callbacks of chains, it'll fiil at the end of analysis
            chains(s).t_callback = []; chains(s).r_callbacks = [];
            if chain_segment_exe_time(s) ~= 0
                callbacks(chain_segment_task_idx(s)).segment_flag = true;
                callbacks(chain_segment_task_idx(s)).segment_C = chain_segment_exe_time(s);
            end
        end
    end
    [~, ind] = sort([executors.id], 'ascend');
    executors = executors(ind);
    
    [~, ind] = sort([callbacks.id], 'ascend');
    callbacks = callbacks(ind);
    
    % compute the WCRT of individual callbacks
    for i = 1 : length(callbacks)
        flag = true;
        
        % cb id
        t_id = callbacks(i).id;
        
        % check segment task
        segment_flag = callbacks(i).segment_flag;

        % executor of target callback
        t_exe = callbacks(i).executor;

        % priority of target callback
        t_prio = callbacks(i).priority;
        
        % chain of target callback
        t_chain = callbacks(i).chain_id;
        
        % chain on cpu
        t_chain_cpu = callbacks(i).chain_on_cpu;
        
        % cpu
        t_cpu = callbacks(i).cpu;
        
        % blocking time by lower priority tasks within executor
        B = 0;
        for c = 1 : length(executors(t_exe).callbacks)
            cb = executors(t_exe).callbacks(c);
            if cb.chain_id ~= t_chain && cb.priority < t_prio
                if cb.C > B
                    B = cb.C;
                end
            end
        end
        
        % initial R
        if segment_flag
            R = callbacks(i).segment_C + B;
        else
            R = callbacks(i).C + B;
        end
             
        R_prev = R;        
        while flag
            W = 0;
            for j = 1 : length(callbacks)
                % only consider callback on higher- or same priority executor 
                if callbacks(j).id ~= t_id && executors(t_exe).priority <= executors(callbacks(j).executor).priority
                    % check current chain is on a single cpu
                    if ((t_chain_cpu && callbacks(j).chain_id ~= t_chain) && (callbacks(j).cpu == t_cpu)) || ...
                        ((~t_chain_cpu) && (callbacks(j).cpu == t_cpu))
                        [timer_prio, timer_P, timer_cpu] = find_timer_callback(executors, callbacks(j).chain_id);                        
                        if callbacks(j).chain_on_cpu
                            P = max(callbacks(j).chain_c, timer_P);
                        else
                            P = timer_P;
                        end
                        
                        if (executors(t_exe).priority < executors(callbacks(j).executor).priority) || (t_prio < callbacks(j).priority)
                            if timer_prio >= t_prio || timer_cpu ~= t_cpu
                                W = W + ceil(R/P)*callbacks(j).C;
                            else
                                W = W + callbacks(j).C;
                            end
                        end
                    end
                end
            end
            
            if segment_flag
                R = W + callbacks(i).segment_C + B;
            else
                R = W + callbacks(i).C + B;
            end
            
            if R <= R_prev
                callbacks(i).wcrt = R;
                break;
            end
            R_prev = R;
        end
    end
    
    % reshape callbacks to chains where they belong to.
    for c = 1 : length(callbacks)
        if strcmp(callbacks(c).type, 'timer')
            chains(callbacks(c).chain_id).t_callback = callbacks(c);
        else
            chains(callbacks(c).chain_id).r_callbacks = [chains(callbacks(c).chain_id).r_callbacks callbacks(c)];
        end
    end
    
    % Theorem 1.
    % capture WCRT with considering time delay by prior chain instance
    chain_latency = zeros(1, length(chains));
    for c = 1 : length(chains)
        if chains(c).t_callback.segment_flag
            chain_latency(c) = chain_latency(c) + chains(c).t_callback.wcrt;
        end
        for t = 1 : length(chains(c).r_callbacks)
            if chains(c).r_callbacks(t).segment_flag
                chain_latency(c) = chain_latency(c) + chains(c).r_callbacks(t).wcrt;
            end 
        end
        
        if chain_latency(c) > chains(c).t_callback.T
            chain_latency(c) = chain_latency(c) + chains(c).t_callback.T;
        end
    end
    
    
end

function [timer_prio timer_P timer_cpu] = find_timer_callback(executors, chain_id)
    for e = 1 : length(executors)
        for c = 1 : length(executors(e).callbacks)
            cb = executors(e).callbacks(c);
            if cb.chain_id == chain_id && strcmp(cb.type, 'timer')
                timer_prio = cb.priority;
                timer_P = cb.T;
                timer_cpu = cb.cpu;
                return;
            end
        end
    end
end