%% Schedulability test (Section VII-C)
clc; clear all; close all;

files = ["cpu4_util25_chain9_callback45.txt", "cpu4_util30_chain9_callback45.txt", ...
            "cpu4_util35_chain9_callback45.txt"];

eval_running_time = [];
for n = 1 : length(files)
    data_raw = load(files(n));
    num_tasks_sets = 45;
    num_sets = 1000;
    measured_taskset = zeros(num_sets, 1);
    
    latency_chains = [];
    schedulability_chains = [];

    s = 1;
    for k = 1 : num_sets
        data = data_raw(s:s+num_tasks_sets-1, :);
        num_executors = max(data(:, 7));
        num_cpus = max(data(:, 6));
        num_chains = max(data(:, 3));
        num_tasks = size(data, 1);
        if num_tasks_sets ~= num_tasks
            error('Number of tasks for a set is different.');
        end


        % initialize callbacks & chains
        chains = []; chain_idx = 0;
        sem_prio = num_chains;
        for c = 1 : size(data, 1)
            callbacks(c) = Callback(c, data(c, 1), data(c, 2), data(c, 3), data(c, 4), data(c, 6), data(c, 7));
            if length(chains) < callbacks(c).chain_id
                chain_idx = chain_idx+ 1;
                chains = [chains Chain(chain_idx, sem_prio)];
                sem_prio = sem_prio - 1;
            end
            chains(callbacks(c).chain_id) = chains(callbacks(c).chain_id).add_callback(callbacks(c));    
        end

        % assign callback priority
        callback_prio = num_tasks;
        callbacks_sorted = [];
        for c = 1 : num_chains
            for t = 1 : chains(c).num_callbacks-1
                chains(c).r_callbacks(chains(c).num_callbacks-t).priority = callback_prio;
                callback_prio = callback_prio - 1;
                chains(c).r_callbacks(chains(c).num_callbacks-t).chain_T = chains(c).T;
                callbacks_sorted = [callbacks_sorted chains(c).r_callbacks(chains(c).num_callbacks-t)];
            end
            chains(c).t_callback.priority = callback_prio;
            callback_prio = callback_prio - 1;
            chains(c).t_callback.chain_T = chains(c).T;
            callbacks_sorted = [callbacks_sorted chains(c).t_callback];
        end

        [~, ind] = sort([callbacks_sorted.id], 'ascend');
        callbacks = callbacks_sorted(ind);

        for i = 1 : num_chains
            [idx, ~] = find(data(:,3) == i);
            % check chain_on_cpu
            tmp_cpu = data(idx(1), 6);
            check_on_cpu = true;
            for j = 2 : length(idx)
                if tmp_cpu ~= data(idx(j), 6)
                    check_on_cpu = false;
                end
            end

            if check_on_cpu
                for j = 1 : length(idx)
                    callbacks(idx(j)).chain_on_cpu = true;
                end        
            end
        end

        % initialize executors
        prio = num_executors;
        for e = 1 : num_executors
            executors(e) = Executor(e, prio);
            prio = prio - 1;
        end

        % initialize cpus
        for c = 1 : num_cpus
            cpus(c) = Cpu(c);
        end

        % assign callbacks to executors
        for i = 1 : num_tasks
            executors(callbacks(i).executor) = executors(callbacks(i).executor).add_callbacks(callbacks(i));    
        end

        for i = 1 : num_executors
            tmp_cpu = executors(i).callbacks(1).cpu;
            for j = 1 : length(executors(i).callbacks)
                if executors(i).callbacks(j).cpu ~= tmp_cpu
                    error('An executor has multiple cpu configuration.');
                end
            end
            cpus(tmp_cpu) = cpus(tmp_cpu).assign_executor(executors(i));
        end
        
        % compute response time of callbacks
        tic;
        [chains, latency] = response_time_callbacks(chains, cpus);
        measured_taskset(k) = toc;
        
        latency_chains = [latency_chains; latency];
        tmp_sched = [];
        for i = 1 : num_chains
            sched = 0;
            if latency(i) < chains(i).C
                error('Latency is less than execution time.');
            end
            if latency(i) <= chains(i).T
                sched = 1;
            end
            tmp_sched = [tmp_sched, sched];
        end
        schedulability_chains = [schedulability_chains; tmp_sched];

        s = s + num_tasks_sets;
        
    end
    eval_running_time = [eval_running_time, measured_taskset];
    mean_latency = mean(latency_chains);
    sched_ratio = sum(schedulability_chains)./num_sets;
end

disp(['Average running times for ', num2str(num_sets), ' sets are ', num2str(mean(eval_running_time))]);
save('picas_running_time.txt', 'eval_running_time', '-ascii');
