function robots = updateParticles(robots,map,depot,obsDist,k,goalReached)
numBots = size(robots(k).particles,1);
numParticles = size(robots(k).particles,2);
for r = 1:numBots
    for p = 1:numParticles
        %If connected update particle guesses to current pose and
        %speed
        if ~isempty(robots(k).particles(r,p).plan) 
            thisParticle = robots(k).particles(r,p);
           %Is a task complete? Check if tasks should be added or are complete
            if norm(thisParticle.pose(1:2)-thisParticle.plan(1,1:2))<goalReached
                robots(k).particles(r,p).plan(1,:) = [];
                % robots(k).particles(r,p).taskQueue(1) = [];
                if thisParticle.plan(1,4) == 0
                    robots(k).particles(r,p).taskQueue(1) = [];
                end
            end
            if ~isempty(robots(k).particles(r,p).plan)
                robots(k).particles(r,p).goalLoc = robots(k).particles(r,p).plan(1,1:2);
                goal = robots(k).particles(r,p).goalLoc;
            else
                goal = depot;
            end
        else
            goal = depot;
        end
        [pose,~] = move_particle_forces(robots(k).particles(r,p),map, ...
                                goal,obsDist);
        robots(k).particles(r,p).pose = pose;
       

    end %numparticles
end %numbots

end %function