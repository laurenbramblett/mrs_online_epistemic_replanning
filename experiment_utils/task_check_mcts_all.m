%Comms  Check
conBots = find(bins == bins(k)); 
opBots = find(bins == bins(k) & robots(k).deadBots'<1); %Find connected agents who aren't dead

%Who do we expect to be there

if robots(k).max_v>0
for r = 1:numBots
    if k~=r
        planType = robots(k).replan;
        %Which ones have I accounted for but am not assigned
        particleIdx = min(robots(k).particleGuess(r),numParticles);
        otherPose = robots(k).particles(r,particleIdx).pose(1:2);
        %Who isn't here and I havent planned for
        if norm(robots(k).pose(1:2)-otherPose)<robots(k).maxRange*2-2 && bins(k)~=bins(r) && ...
            sum(robots(k).commQueue == r)<1 && sum(robots(k).dfwm == r)<1 
            robots(k).replan = 2; %Start with replan as if already in our queue
            %Add to particle guess
            robots(k).particleGuess(r) = robots(k).particleGuess(r) + 1;
            robots(k).particleLastUpdate(r) = i;
            robots(k).commQueue = [robots(k).commQueue r]; %Add to queue
            % robots(k).dfwm = []; %fresh start
 
            %Any robots that cannot be found through particles
            if robots(k).particleGuess(r)>numParticles
                robots(k).deadBots(r) = 1;
                robots(k).replan = 2;
            end   
        elseif norm(robots(k).pose(1:2)-otherPose)<robots(k).maxRange*2-2 && bins(k)~=bins(r) && ...
                sum(robots(k).commQueue == r)>0 && robots(k).deadBots(r)<1 && sum(robots(k).dfwm == r)<1 
            robots(k).replan = 2; %Already in queue but need new locations
            %Add to particle guess
            robots(k).particleLastUpdate(r) = i;
            robots(k).particleGuess(r) = robots(k).particleGuess(r) + 1;
 
            %Any robots that cannot be found through particles
            if robots(k).particleGuess(r)>numParticles
                robots(k).deadBots(r) = 1;
            end   
            if norm(robots(k).pose(1:2)-robots(k).depot)<robots(k).maxRange*2
                plannedForAtDepot = unique([plannedForAtDepot r conBots]);
            end
        %If connected and I am supposed to connect
        elseif bins(k)==bins(r) && sum(robots(k).commQueue == r)>0
            robots(k).replan = 2;
            robots(k).commQueue(robots(k).commQueue == r) = [];
            whichPlan = robots(k).plan(:,3) == r & robots(k).plan(:,4)>0;
            if ~isempty(whichPlan)
                robots(k).plan(whichPlan,:) = [];
            end
        elseif bins(k)==bins(r) && sum(robots(k).rzQueue == r)>0
            robots(k).replan = 2;
            rzIdx = robots(k).rzQueue == r;
            robots(k).rzQueue(rzIdx) = [];
            whichPlan = robots(k).plan(:,3) == r & robots(k).plan(:,5)>0;
            if ~isempty(whichPlan)
                robots(k).plan(whichPlan,:) = [];
            end
        elseif bins(k)==bins(r) && sum(robots(k).dfwm == r)<1
            whichFailure = i>=whenFail; 
            if any(r==whoFail(whichFailure)) && ~isempty(robots(r).taskQueue) && ~isequal(robots(r).plan(1,1:2),robots(r).depot) && ...
                    robots(k).deadBots(r)<1
                robots(k).replan = 2;
                robots(k).dfwm = unique([robots(k).dfwm r]);
                if robots(r).max_v == 0 
                    robots(k).deadBots(r)=1;
                    opBots = find(bins == bins(k) & robots(k).deadBots'<1); %Find connected agents who aren't dead
                    robots(k).replan = 2;
                end
            end
            if norm(robots(k).pose(1:2)-robots(k).depot)<robots(k).maxRange*2
                plannedForAtDepot = unique([plannedForAtDepot conBots]);
            end
        end

        if norm(robots(k).pose(1:2)-robots(k).depot)<robots(k).maxRange*2 && sum(plannedForAtDepot == r)<1
            plannedForAtDepot = unique([plannedForAtDepot conBots]);
        elseif sum(plannedForAtDepot == r)>0 
            % robots(k).replan = planType;
        end
    end
end
whichOp = find(sum([robots(conBots).deadBots],2)<1);
%Don't replan redundantly
if isequal(conBots,whichOp') && robots(k).connected_replan == 0 && sum(sum([robots(:).deadBots]))>0
    robots(k).replan = 2;
    for j = 1:length(conBots)
        robots(conBots(j)).connected_replan = 1;
    end
elseif ~isequal(conBots,whichOp')
    for j = 1:length(conBots)
        robots(conBots(j)).connected_replan = 0;
    end
end
end

% robots(k).commQueue = unique([robots(conBots).commQueue]);
% robots(k).rzQueue = unique([robots(conBots).rzQueue]);
if isempty(robots(2).commQueue) && i > 900
    % keyboard
end
if sum([robots(conBots).replan])<1
    for a1 = 1:length(conBots)
        if ~isempty(robots(conBots(a1)).dfwm)
            for a2 = 1:length(conBots)
                if isempty(robots(conBots(a2)).commQueue) && isempty(robots(conBots(a2)).rzQueue)
                    robots(conBots(a2)).dfwm = unique([robots(conBots).dfwm]);
                end
            end
        end
    end
end

