%connected_logic
if connected
    for k = 1:numBots
        robots(k).particleFollow = 1;
        robots(k).particleGuess = ones(numBots,1);
        robots(k).dfwm = [];
        % robots(k).commQueue = [];
        % robots(k).rzQueue = [];
        % robots(k).deadBots = min(robots);
        for p = 1:numParticles
            for f = fn
                robots(k).particles(k,p).(f{1}) = robots(k).(f{1});
            end
            robots(k).particles(k,p).max_v = robots(k).max_v*(1-(p-1)*decay);
        end
    end
end

for k1 = 1:numBots
    for k2 = 1:numBots
        if ~isequal(robots(k2).plan,robots(k2).particles(k2,1).plan)
            robots(k2).iamdifferent = true;
        end
        
        if robots(k1).max_v == 0
            robots(k1).deadBots(k1) = 1;
        else
            robots(k1).deadBots(k1) = 0;
        end

        if bins(k1) == bins(k2) && k1~=k2
            if robots(k1).max_v>0 && robots(k2).deadBots(k1) 
                robots(k2).deadBots(k1) = 0;
                robots(k1).replan = 2;
            elseif robots(k1).max_v == 0 && ~robots(k2).deadBots(k1)
                robots(k2).deadBots(k1) = 1;
                robots(k2).replan = 2;
            end
            robots(k1).robotLastUpdate(k2) = i;
            robots(k1).particleLastUpdate(k2) = i;
            robots(k1).tasks_done = unique([robots(k1).tasks_done robots(k2).tasks_done]);

            if sum(robots(k1).commQueue == k2)>0
                commIdx = robots(k1).commQueue == k2;
                robots(k1).commQueue(commIdx) = [];  
                robots(k1).replan = 2;
                if robots(k2).max_v == 0
                    robots(k1).deadBots(k2) = 1;
                    robots(k1).replan = 2;
                end
                robots(k1).dfwm = unique([robots(k1).dfwm k2]);
            elseif sum(robots(k1).rzQueue == k2)>0
                rzIdx = robots(k1).rzQueue == k2;
                robots(k1).rzQueue(rzIdx) = [];
                robots(k1).replan = 2;
                robots(k1).dfwm = unique([robots(k1).dfwm k2]);
            end

            if robots(k2).iamdifferent || connected
                for p = 1:numParticles
                    for f = fn
                        robots(k1).particles(k2,p).(f{1}) = robots(k2).(f{1});
                    end
                    robots(k1).particles(k2,p).max_v = robots(k2).max_v*(1-(p-1)*decay);
                end
            end
        end
        for k3 = 1:numBots
            if k3~=k1 && k3~=k2  && bins(k1) == bins(k2) && k1 ~= k2
                if robots(k1).robotLastUpdate(k3)<robots(k2).robotLastUpdate(k3)
                    robots(k1).robotLastUpdate(k3) = robots(k2).robotLastUpdate(k3);
                    robots(k1).particleLastUpdate(k3) = robots(k2).particleLastUpdate(k3);
                    if robots(k2).deadBots(k3) == 1
                        robots(k1).deadBots(k3) = 1;
                        if sum(robots(k1).commQueue == k3)>0
                            commIdx = robots(k1).commQueue == k3;
                            robots(k1).commQueue(commIdx) = [];  
                            robots(k1).replan = 2;
                            robots(k1).dfwm = unique([robots(k1).dfwm k3]);
                        elseif sum(robots(k1).rzQueue == k3)>0
                            rzIdx = robots(k1).rzQueue == k3;
                            robots(k1).rzQueue(rzIdx) = [];
                            robots(k1).replan = 2;
                            robots(k1).dfwm = unique([robots(k1).dfwm k3]);
                        end
                    end
                    for p = 1:numParticles
                        robots(k1).particles(k3,p) = robots(k2).particles(k3,p);
                    end
                elseif robots(k1).particleLastUpdate(k3)<robots(k2).particleLastUpdate(k3)
                    robots(k1).particleLastUpdate(k3) = robots(k2).particleLastUpdate(k3);
                    robots(k1).particleGuess(k3) = robots(k2).particleGuess(k3);
                end
            end
        end
    end
end