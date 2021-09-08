%% Case study (Section VII-B)
clc; clear all; close all;

% Period(only for timer callback), Execution time, Deadline(only for timer callback), Chain, Order in chain
% Case study III (overloeaded scenario)
data = [80 2.3 80 1 1;
        0 16.1 0 1 2;
        80 2.3 80 2 1;
        0 2.2 0 2 2;
        0 18.4 0 2 3;
        0 9.1 0 2 4;
        100 23.1 100 3 1;
        0 7.9 0 3 2;
        0 14.2 0 3 3;
        0 17.9 0 3 4;
        100 20.6 100 4 1;
        0 17.9 0 4 2;
        0 6.6 0 4 3;
        160 1.7 160 5 1;
        0 11 0 5 2;
        0 6.6 0 5 3;
        0 7.9 0 5 4;
        1000 1.7 1000 6 1;
        0 195.9 0 6 2;
        120 33.2 120 7 1;
        0 2.2 0 7 2;
        120 33.2 120 8 1;
        0 6.6 0 8 2;
        120 33.2 120 9 1;
        0 6.6 0 9 2;
        120 33.2 120 10 1;
        0 1.7 0 10 2;
        120 33.2 120 11 1;
        0 2.2 0 11 2;
        120 33.2 120 12 1;
        0 2.2 0 12 2];
        
num_executors = 18;
num_cpus = 4;
num_chains = max(data(:, 4));
num_tasks = size(data, 1);

% Initialize callbacks & chains
chains = []; chain_idx = 0;
sem_prio = num_chains;
for c = 1 : size(data, 1)
    callbacks(c) = Callback(c, data(c, 1), data(c, 2), data(c, 4), data(c, 5));
    if length(chains) < callbacks(c).chain_id
        chain_idx = chain_idx+ 1;
        chains = [chains Chain(chain_idx, sem_prio)];
        sem_prio = sem_prio - 1;
    end
    chains(callbacks(c).chain_id) = chains(callbacks(c).chain_id).add_callback(callbacks(c));    
end

% Initialize executors
prio = num_executors;
for e = 1 : num_executors
    executors(e) = Executor(e, prio);
    prio = prio - 1;
end

% Initialize cpus
for c = 1 : num_cpus
    cpus(c) = Cpu(c);
end

% Assign callback priority
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

callbacks(1).priority = callbacks(3).priority;  % Since callback(1) and callback(3) are the same, i.e., a mutual callback

% If all callbacks of a chain exist on the same CPU core, 
% set "chain_on_cpu" True
callbacks(1).chain_on_cpu = true;
callbacks(2).chain_on_cpu = true;
callbacks(7).chain_on_cpu = true;
callbacks(8).chain_on_cpu = true;
callbacks(9).chain_on_cpu = true;
callbacks(10).chain_on_cpu = true;
callbacks(11).chain_on_cpu = true;
callbacks(12).chain_on_cpu = true;
callbacks(13).chain_on_cpu = true;
callbacks(14).chain_on_cpu = true;
callbacks(15).chain_on_cpu = true;
callbacks(16).chain_on_cpu = true;
callbacks(17).chain_on_cpu = true;
callbacks(18).chain_on_cpu = true;
callbacks(19).chain_on_cpu = true;

% Allocate callbacks to executors manually
% RT chains
executors(1) = executors(1).add_callbacks(callbacks(1:2));
executors(1) = executors(1).add_callbacks(callbacks(3));
executors(2) = executors(2).add_callbacks(callbacks(4:6));
executors(3) = executors(3).add_callbacks(callbacks(7:10));
executors(4) = executors(4).add_callbacks(callbacks(11:13));
executors(5) = executors(5).add_callbacks(callbacks(14:17));
executors(6) = executors(6).add_callbacks(callbacks(18:19));
% BE chains
executors(7) = executors(7).add_callbacks(callbacks(20));
executors(8) = executors(8).add_callbacks(callbacks(21));
executors(9) = executors(9).add_callbacks(callbacks(22));
executors(10) = executors(10).add_callbacks(callbacks(23));
executors(11) = executors(11).add_callbacks(callbacks(24));
executors(12) = executors(12).add_callbacks(callbacks(25));
executors(13) = executors(13).add_callbacks(callbacks(26));
executors(14) = executors(14).add_callbacks(callbacks(27));
executors(15) = executors(15).add_callbacks(callbacks(28));
executors(16) = executors(16).add_callbacks(callbacks(29));
executors(17) = executors(17).add_callbacks(callbacks(30));
executors(18) = executors(18).add_callbacks(callbacks(31));

% Allocate executors to CPUs
cpus(1) = cpus(1).assign_executor(executors(1));
cpus(1) = cpus(1).assign_executor(executors(5));
cpus(1) = cpus(1).assign_executor(executors(11));
cpus(1) = cpus(1).assign_executor(executors(14));
cpus(1) = cpus(1).assign_executor(executors(15));

cpus(2) = cpus(2).assign_executor(executors(2));
cpus(2) = cpus(2).assign_executor(executors(8));
cpus(2) = cpus(2).assign_executor(executors(9));
cpus(2) = cpus(2).assign_executor(executors(16));
cpus(2) = cpus(2).assign_executor(executors(17));

cpus(3) = cpus(3).assign_executor(executors(3));
cpus(3) = cpus(3).assign_executor(executors(7));
cpus(3) = cpus(3).assign_executor(executors(12));
cpus(3) = cpus(3).assign_executor(executors(18));

cpus(4) = cpus(4).assign_executor(executors(4));
cpus(4) = cpus(4).assign_executor(executors(6));
cpus(4) = cpus(4).assign_executor(executors(13));
cpus(4) = cpus(4).assign_executor(executors(10));

% Compute response time of callbacks
[chains, latency] = response_time_callbacks(chains, cpus);



