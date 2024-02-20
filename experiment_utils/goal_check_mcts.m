% Find goal for a robot

if isempty(robots(k).plan)
    robots(k).goalLoc = robots(k).depot;
    robots(k).state = "Do Depot";
else
    robots(k).goalLoc = robots(k).plan(1,1:2);
    isTask = robots(k).plan(1,4) == 0;
    isRz = robots(k).plan(1,5) > 0;
    isComm = ~isTask & ~isRz;
    if isTask
        robots(k).state = "Do Tasks";
    elseif isRz
        robots(k).state = "Do Rz";
    elseif isComm
        robots(k).state = "Do Comm";
    end
end

distToTasks = vecnorm(robots(k).pose(1:2)-robots(k).globalTaskLocs,2,2);
if any(distToTasks<goalReached)
    whichClose = find(distToTasks<goalReached+0.2);
    % whichClose = whichClose(ismember(whichClose,robots(k).taskQueue));
    robots(k).tasks_done = unique([robots(k).tasks_done whichClose']);
    if any(taskColorStore(whichClose)<1)
        taskColorStore(whichClose) = k;
    end
end

%Check if actually arrived at belief or movement error requires small
%perturbation
%Plan is current location(1:2), robot index (3), particle (4), min rz time (5 - when applicable > 0)

if robots(k).state == "Do Rz" || robots(k).state == "Do Comm"
    if norm(robots(k).goalLoc-robots(k).pose(1:2))<goalReached
        whichBot = robots(k).plan(1,3); whichParticle = robots(k).plan(1,4);
        if whichParticle<=numParticles
            pB = robots(k).particles(whichBot,whichParticle);
            whichLoc = robots(k).particles(whichBot,whichParticle).pose(1:2);
            if norm(robots(k).pose(1:2)-whichLoc)<robots(k).maxRange*2-2
                robots(k).plan(1,:) = [];
            else
                velA = robots(k).max_v; velB = pB.max_v;
                if isempty(pB.plan)
                    pB.plan = robots(k).depot;
                end
                pt = findIntersectPoint(robots(k).pose(1:2),whichLoc,pB.plan(:,1:2),...
                    velA,velB,dt,goalReached,robots(k).maxRange*2-2,i*dt,i*dt,pB.depot,pB);
                robots(k).plan(1,1:2) = pt;
                robots(k).goalLoc = plan(1,1:2);
            end
        else
            robots(k).plan(1,:) = [];
            if isempty(robots(k).plan(:,3) == whichBot)
                commIdx = robots(k).commQueue == whichBot;
                robots(k).commQueue(commIdx) = [];
                robots(k).dfwm = unique([robots(k).dfwm whichBot]);
            end
        end
    elseif sum(robots(k).plan(3) == robots(k).dfwm)>0
        robots(k).plan(1,:) = [];
        if ~isempty(robots(k).plan)
            robots(k).goalLoc = robots(k).plan(1,:);
        else
            robots(k).goalLoc = robots(k).depot;
        end
    end
elseif norm(robots(k).goalLoc-robots(k).pose(1:2))<goalReached && robots(k).state ~= "Do Depot"
    robots(k).plan(1,:) = [];
    robots(k).taskQueue(1) = [];
    if ~isempty(robots(k).plan)
        robots(k).goalLoc = robots(k).plan(1,1:2);
    else
        robots(k).goalLoc = robots(k).depot;
    end
elseif ~isempty(robots(k).plan)
    if sum(robots(k).plan(1,3) == robots(k).tasks_done)>0
        robots(k).plan(1,:) = [];
        robots(k).taskQueue(1) = [];
        if ~isempty(robots(k).plan)
            robots(k).goalLoc = robots(k).plan(1,1:2);
        else
            robots(k).goalLoc = robots(k).depot;
        end
    end
end

if ~robots(k).reachedNewParticle
    newGoal = robots(k).particles(k,robots(k).particleFollow).pose(1:2);
    robots(k).goalLoc = newGoal;
    if norm(robots(k).goalLoc-robots(k).pose(1:2))<goalReached
        robots(k).reachedNewParticle = true;
    end
end




% %Update goal if anything changed from above
% if isempty(robots(k).rzQueue)
%     if ~isempty(robots(k).commQueue)
%         bot = robots(k).commQueue(1);
%         if robots(k).deadBots(bot)<1
%             robots(k).particles(bot,robots(k).particleGuess(bot)).pose(1:2)
%         else
%             robots(k).goalLoc = robots(k).commLocs(1,1:2);
%         end
%     end
% elseif ~isempty(robots(k).rzQueue)
%     bot = robots(k).rzQueue(1);
%     robots(k).goalLoc = robots(k).particles(bot,1).pose(1:2);
% elseif ~isempty(robots(k).commQueue)
%     bot = robots(k).commQueue(1);
%     if robots(k).deadBots(bot)<1
%         robots(k).particles(bot,robots(k).particleGuess(bot)).pose(1:2)
%     else
%         robots(k).goalLoc = robots(k).commLocs(1,1:2);
%     end
% end

