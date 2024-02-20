%Check if something must change
%Find rz instances
if robots(k).replan == 1 
    deadBots = find(sum([robots(conBots).deadBots],2)>0);
    [botsComm,minTimes,rzLoc] = findMinTimes(timeWindows,i*dt,[conBots,robots(k).commQueue,robots(k).dfwm,deadBots']);
    %Find who to look for
    botsLost = robots(k).commQueue';
    lostLocs = zeros(length(botsLost),2);
    for j = 1:length(botsLost)
        whichBot = botsLost(j);
        %Is robot dead?
        ded = robots(k).deadBots(whichBot) || robots(k).particleGuess(whichBot)>numParticles;
        if ded
            lostLocs(j,:) = [1e4,1e4]; % Constant for now. Could replace with total dist of path or bounds of env
        else
            if ~isempty(robots(k).particles(whichBot,robots(k).particleGuess(whichBot)).plan(:,1:2))
                lostLocs(j,:) = mean(robots(k).particles(whichBot,robots(k).particleGuess(whichBot)).plan(:,1:2),1);
            else
                lostLocs(j,:) = robots(k).depot;
            end
        end
    end
    
    %Partition assignments if connected
    taskCentroid = zeros(length(opBots),2);
    for ops = 1:length(opBots)
        whichPlan = [robots(opBots(ops)).pose(1:2);robots(opBots(ops)).plan(:,1:2)];
        taskCentroid(ops,:) = mean(whichPlan,1);
    end
    newTaskLocs = [rzLoc; lostLocs]; 
    newTaskBots = [botsComm;botsLost];
    %Dictionary for task assignment and dynamic tsp reward
    %Dict is current location(1:2), robot index (3), particle (4), min rz time (5 - when applicable > 0)
    newTaskDict = [newTaskLocs,newTaskBots,zeros(length(newTaskBots),2)]; %Add min time to rz and 
    if ~isempty(botsComm)
        newTaskDict(1:length(botsComm),5) = minTimes;
    end
    
    for tNew = 1:size(newTaskLocs,1)
        whichParticle = robots(k).particleGuess(newTaskDict(tNew,3));
        newTaskDict(tNew,4) = whichParticle;
    end
    robot_locs = reshape([robots(opBots).pose]',3,[])';
    robot_speeds = [robots(opBots).max_v]';
    partition = zeros(size(newTaskDict,1),1); cost = zeros(length(opBots));
    for tNew = 1:size(newTaskLocs,1)
        tLoc = newTaskDict(tNew,:);
        [addTime,assIdx,robot_locs] = compute_partition(robots(k),robot_speeds,robot_locs,tLoc,cost);
        partition(tNew) = opBots(assIdx); 
        cost(assIdx) = cost(assIdx) + addTime(assIdx);
        partition(tNew) = opBots(assIdx); 
        notAssigned = my_setdiff(conBots,opBots(assIdx));
        for na = 1:length(notAssigned)
            robots(notAssigned(na)).dfwm(end+1) = newTaskBots(tNew);
            robots(notAssigned(na)).dfwm(end+1) = opBots(assIdx);
        end
    end
    continue_plan = 0; numMCTS_Iters = 1400; full_replan = 0;
    [root, best_tour, tour_states, tasksAssigned] = mcts_planning(numMCTS_Iters,...
        robots,newTaskDict,partition,opBots,i,continue_plan,full_replan);

    for ops = 1:length(opBots)
        robots(opBots(ops)).deadBots(deadBots) = 1;
        robots(opBots(ops)).root = root{ops};
        whichTasks = tasksAssigned{ops}(best_tour{ops}(2:end),:);
        whichTasks(:,1:2) = tour_states{ops}(2:end,:);
        robots(opBots(ops)).tasksAssigned = whichTasks;
        robots(opBots(ops)).plan = [whichTasks, (1:size(whichTasks,1))'];
        robots(opBots(ops)).taskQueue = whichTasks(whichTasks(:,4)<1,3)';
        robots(opBots(ops)).rzQueue = whichTasks(whichTasks(:,5)>0,3)';
        robots(opBots(ops)).commQueue = whichTasks(whichTasks(:,4)>0 & whichTasks(:,5)<1,3)';
        robots(opBots(ops)).dfwm = unique([robots(opBots(ops)).dfwm conBots]);
        
        %If assigned a robot that is dead, adjust plan to account for
        %backtracking
        if any(robots(opBots(ops)).deadBots(robots(opBots(ops)).commQueue)>0)
            whichIdx = find(robots(opBots(ops)).deadBots(robots(opBots(ops)).commQueue)>0);
            whichBot = robots(opBots(ops)).commQueue(whichIdx);
            interPlan = [whichTasks,(1:size(robots(opBots(ops)).plan,1))'];
            for b = 1:length(whichBot)
                whichPlan = find(whichTasks(:,3)==whichBot(b) & whichTasks(:,4)>0);
                insertPlace = find(interPlan(:,6)==whichPlan);
                pB = robots(opBots(ops)).particles(whichBot(b),end);
                path_left = flip(my_setdiff(pB.originalPlan,pB.taskQueue));
                path_locs_left = pB.globalTaskLocs(path_left,:);
                taskIdentify = interPlan(insertPlace,3:end);
                path_locs_left = [path_locs_left repelem(taskIdentify,length(path_left),1)];
                interPlan = [interPlan(1:insertPlace-1,:); path_locs_left; interPlan(insertPlace+1:end,:)];
            end
            robots(opBots(ops)).plan = interPlan;
        end
    end
    for r1 = 1:length(conBots)
        for r2 = 1:length(conBots)
            if r1~=r2
                for p = 1:numParticles
                    for f = fn
                        robots(conBots(r1)).particles(conBots(r2),p).(f{1}) = robots(conBots(r2)).(f{1});
                    end
                end
            end
        end
    end
elseif robots(k).replan == 2 && ~isempty(opBots)%Udpate tasks to do with broken guy and connected bots
tasksToDo = unique([robots(conBots).taskQueue]);
tasksDone = [robots(conBots).tasks_done];
tasksToDo = my_setdiff(tasksToDo,tasksDone);
deadBots = find(sum([robots(conBots).deadBots],2)>0);
if ~isempty(tasksToDo) && ~isequal(tasksToDo,size(taskLoc,1))
    [botsComm,minTimes,rzLoc] = findMinTimes(timeWindows,i*dt,[conBots,robots(k).commQueue,deadBots']);
else
    botsComm = []; rzLoc = [];
end
%Find who to look for
botsLost = robots(k).commQueue';
lostLocs = zeros(length(botsLost),2);
for j = 1:length(botsLost)
    whichBot = botsLost(j);
    %Is robot dead?
    ded = robots(k).deadBots(whichBot) || robots(k).particleGuess(whichBot)>numParticles;
    if ded
        lostLocs(j,:) = [1e4,1e4]; % Constant for now. Could replace with total dist of path or bounds of env
    else
        lostLocs(j,:) = mean(robots(k).particles(whichBot,robots(k).particleGuess(whichBot)).plan(:,1:2),1);
    end
end

newTaskLocs = [rzLoc; lostLocs]; 
newTaskBots = [botsComm;botsLost];
%Dictionary for task assignment and dynamic tsp reward
%Dict is current location(1:2), robot index (3), particle (4), min rz time (5 - when applicable > 0)
newTaskDict = [newTaskLocs,newTaskBots,zeros(length(newTaskBots),2)]; %Add min time to rz and 
if ~isempty(botsComm)
    newTaskDict(1:length(botsComm),5) = minTimes;
end
for tNew = 1:size(newTaskLocs,1)
    whichParticle = robots(k).particleGuess(newTaskDict(tNew,3));
    newTaskDict(tNew,4) = whichParticle;
end
%Add all tasks
allTaskDict = [newTaskDict; robots(k).globalTaskLocs(tasksToDo,:),tasksToDo',zeros(length(tasksToDo),2)];
%Voronoi weighted clustering
num_new_tasks = size(allTaskDict,1);
robot_locs = reshape([robots(conBots).pose]',3,[])';
robot_speeds = [robots(conBots).max_v]';
partition = zeros(num_new_tasks,1); cost = zeros(length(conBots),1);
for tNew = 1:num_new_tasks
    tLoc = allTaskDict(tNew,:);
    [addTime,assIdx,robot_locs] = compute_partition(robots(k),robot_speeds,robot_locs,tLoc,cost);
    partition(tNew) = conBots(assIdx); 
    cost(assIdx) = cost(assIdx) + addTime(assIdx);
    partition(tNew) = conBots(assIdx); 
    % knapsack(assignment(i)) = knapsack(assignment(i)) + norm(customers(i,:)-salesmen(assignment(i),:))/speeds(assignment(i));
    if tNew<=length(newTaskBots)
        notAssigned = my_setdiff(conBots,conBots(assIdx));
        for na = 1:length(notAssigned)
            robots(notAssigned(na)).dfwm(end+1) = newTaskBots(tNew);
            robots(notAssigned(na)).dfwm(end+1) = conBots(assIdx);
        end
    end
end
continue_plan = 0; numMCTS_Iters = 4500; full_replan = 1;

    [root, best_tour, tour_states, tasksAssigned] = mcts_planning(numMCTS_Iters,...
        robots,allTaskDict,partition,opBots,i,continue_plan,full_replan);

    for ops = 1:length(opBots)
        robots(opBots(ops)).deadBots(deadBots) = 1;
        robots(opBots(ops)).root = root{ops};
        whichTasks = tasksAssigned{ops}(best_tour{ops}(2:end),:);
        whichTasks(:,1:2) = tour_states{ops}(2:end,:);
        robots(opBots(ops)).tasksAssigned = whichTasks;
        robots(opBots(ops)).plan = [whichTasks, (1:size(whichTasks,1))'];
        robots(opBots(ops)).taskQueue = whichTasks(whichTasks(:,4)<1,3)';
        robots(opBots(ops)).rzQueue = whichTasks(whichTasks(:,5)>0,3)';
        robots(opBots(ops)).commQueue = whichTasks(whichTasks(:,4)>0 & whichTasks(:,5)<1,3)';
        robots(opBots(ops)).dfwm = unique([robots(opBots(ops)).dfwm conBots]);
        for q = 1:length(robots(opBots(ops)).rzQueue)
            if sum(robots(opBots(ops)).rzQueue(q) == robots(opBots(ops)).dfwm)>0
                whichRz = robots(opBots(ops)).rzQueue(q) == robots(opBots(ops)).dfwm;
                robots(opBots(ops)).dfwm(whichRz) = [];
            end
        end
        for q = 1:length(robots(opBots(ops)).commQueue)
            if sum(robots(opBots(ops)).commQueue(q) == robots(opBots(ops)).dfwm)>0
                whichRz = robots(opBots(ops)).commQueue(q) == robots(opBots(ops)).dfwm;
                robots(opBots(ops)).dfwm(whichRz) = [];
            end
        end
        
        %If assigned a robot that is dead, adjust plan to account for
        %backtracking
        if any(robots(opBots(ops)).deadBots(robots(opBots(ops)).commQueue)>0)
            whichIdx = find(robots(opBots(ops)).deadBots(robots(opBots(ops)).commQueue)>0);
            whichBot = robots(opBots(ops)).commQueue(whichIdx);
            interPlan = [whichTasks,(1:size(robots(opBots(ops)).plan,1))'];
            for b = 1:length(whichBot)
                whichPlan = find(whichTasks(:,3)==whichBot(b) & whichTasks(:,4)>0);
                insertPlace = find(interPlan(:,6)==whichPlan);
                pB = robots(opBots(ops)).particles(whichBot(b),end);
                path_left = flip(my_setdiff(pB.originalPlan,pB.taskQueue));
                path_locs_left = pB.globalTaskLocs(path_left,:);
                taskIdentify = interPlan(insertPlace,3:end);
                path_locs_left = [path_locs_left repelem(taskIdentify,length(path_left),1)];
                interPlan = [interPlan(1:insertPlace-1,:); path_locs_left; interPlan(insertPlace+1:end,:)];
            end
            robots(opBots(ops)).plan = interPlan;
        end
    end
    for r1 = 1:length(conBots)
        for r2 = 1:length(conBots)
            if r1~=r2
                for p = 1:numParticles
                    for f = fn
                        robots(conBots(r1)).particles(conBots(r2),p).(f{1}) = robots(conBots(r2)).(f{1});
                    end
                end
            end
        end
    end
elseif ~isempty(robots(k).root)
    % numMCTS_Iters = 200; 
    % changeablePlan = unique(robots(k).plan(robots(k).plan(:,end)>robots(k).plan(1,end),end));
    % robots(k).tasksAssigned = robots(k).tasksAssigned(changeablePlan,:);
    % [root, best_tour, tour_states, tasksAssigned] = mcts_planning(numMCTS_Iters,...
    %     robots,[],[],k,i,1,0);
    % ops = 1;
    % robots(k).root = root{ops}; 
    % whichTasks = tasksAssigned{ops}(best_tour{ops}(2:end),:);
    % whichTasks(:,1:2) = tour_states{ops}(2:end,:);
    % robots(k).tasksAssigned = whichTasks;
    % robots(k).plan = [whichTasks, (1:size(whichTasks,1))'];
    % robots(k).taskQueue = whichTasks(whichTasks(:,4)<1,3)';
    % robots(k).rzQueue = whichTasks(whichTasks(:,5)>0,3)';
    % robots(k).commQueue = whichTasks(whichTasks(:,4)>0 & whichTasks(:,5)<1,3)';
    % 
    % %If assigned a robot that is dead, adjust plan to account for
    % %backtracking
    % if any(robots(k).deadBots(robots(k).commQueue)>0)
    %     whichIdx = find(robots(k).deadBots(robots(k).commQueue)>0);
    %     whichBot = robots(k).commQueue(whichIdx);
    %     interPlan = [whichTasks,(1:size(robots(k).plan,1))'];
    %     for b = 1:length(whichBot)
    %         whichPlan = find(whichTasks(:,3)==whichBot(b));
    %         insertPlace = find(interPlan(:,6)==whichPlan);
    %         pB = robots(k).particles(whichBot(b),end);
    %         path_left = flip(my_setdiff(pB.originalPlan,pB.taskQueue));
    %         path_locs_left = pB.globalTaskLocs(path_left,:);
    %         taskIdentify = interPlan(insertPlace,3:end);
    %         path_locs_left = [path_locs_left repelem(taskIdentify,length(path_left),1)];
    %         interPlan = [interPlan(1:insertPlace-1,:); path_locs_left; interPlan(insertPlace+1:end,:)];
    %     end
    %     robots(k).plan = interPlan;
    % end

end
if robots(k).replan>0
    for a = 1:length(conBots)
        robots(conBots(a)).replan = 0;
    end
end

if isempty(robots(k).plan)
    robots(k).state = "Do Depot";
else
    robots(k).state = "Do Tasks";
end
% elseif isempty(robots(k).rzQueue) && isempty(robots(k).commQueue) 
%     robots(k).state = "Do Tasks";
% elseif ~isempty(robots(k).rzQueue)
%     robots(k).state = "Do Rz";
% elseif ~isempty(robots(k).commQueue)
%     robots(k).state = "Do Comm";


