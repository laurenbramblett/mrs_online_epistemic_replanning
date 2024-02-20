function [tour_states,total_reward] = compute_tour_reward(tour,tasks,time,robot)
    total_reward = 0;
    total_time = time;
    current_loc = robot.pose(1:2);
    tour_states = current_loc;
    for i = 1:length(tour)-1
        [reward,total_time,current_loc] = ...
            policy_reward(current_loc, tour(i+1),time,total_time,tasks,robot);
        total_reward = total_reward + reward;
        tour_states(end+1,:) = current_loc;
    end
    
    % Close the loop (traveling back to the depot)
    total_reward = total_reward + norm(current_loc-robot.globalTaskLocs(end,:))/robot.max_v;
    % total_length = total_length + distances(tour(end), tour(1),locations);
end

function [reward,time,next_loc] = policy_reward(current_loc,taskB,start_time,total_time,tasks,robot)
    time = total_time; next_loc = current_loc;
    if tasks(taskB,5)>0 && total_time < tasks(taskB,5) 
        whichBot = tasks(taskB,3); whichParticle = tasks(taskB,4);
        pB = robot.particles(whichBot,whichParticle);
        if isempty(pB.plan)
            taskLocsB = robot.depot;
        else
            taskLocsB = pB.plan;
        end
        velB = pB.max_v; velA = robot.max_v;
        nextCityLoc = findIntersectPoint(current_loc,pB.pose(1:2),...
            taskLocsB,velA,velB,robot.dt,1,robot.maxRange,start_time,total_time,pB.depot,pB);
        timeToNext = norm(nextCityLoc-current_loc)/robot.max_v;
        if total_time + timeToNext > tasks(taskB,5)
            reward = 1e6;
        else
            reward = timeToNext;
            time = time + timeToNext;
            next_loc = nextCityLoc;
        end
    elseif tasks(taskB,5)>0
        reward = 1e6;
    elseif tasks(taskB,3)>0 && tasks(taskB,4)>0 && robot.numParticles(tasks(taskB,3))<tasks(taskB,4)
        whichBot = tasks(taskB,3);
        pB = robot.particles(whichBot,end);
        path_left = flip(my_setdiff(pB.originalPlan,pB.taskQueue));

        cost = 0;
        for p = 1:length(path_left)-1
            cost = cost + norm(robot.globalTaskLocs(path_left(p+1),:)-robot.globalTaskLocs(path_left(p),:))/robot.max_v;
        end
        time = time + cost;
        reward = cost;
        if ~isempty(path_left)
            next_loc = robot.globalTaskLocs(path_left(1),:);
        else
            next_loc = robot.depot;
        end
    elseif tasks(taskB,3)>0 && tasks(taskB,4)>0
        whichBot = tasks(taskB,3); whichParticle = tasks(taskB,4);
        pB = robot.particles(whichBot,whichParticle);
        if isempty(pB.plan)
            taskLocsB = robot.depot;
        else
            taskLocsB = pB.plan;
        end
        velB = pB.max_v; velA = robot.max_v;
        nextCityLoc = findIntersectPoint(robot.pose(1:2),pB.pose(1:2),...
            taskLocsB,velA,velB,robot.dt,1,robot.maxRange,start_time,total_time,pB.depot,pB);
        timeToNext = norm(nextCityLoc-current_loc)/robot.max_v;
        reward = timeToNext;
        time = time + timeToNext;
        next_loc = nextCityLoc;
    else
        nextCityLoc = tasks(taskB,1:2);
        timeToNext = norm(nextCityLoc-current_loc)/robot.max_v;
        reward = timeToNext;
        time = time + timeToNext;
        next_loc = nextCityLoc;
    end
end