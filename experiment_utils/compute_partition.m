function [timeForTask,assIdx,newLoc] = compute_partition(robot,speeds,locs,task,cost)
for i = 1:size(locs,1)
    if sum(task(1:2))>1e3
        whichRobot = task(3);
        pB = robot.particles(whichRobot,end);
        taskLeft = my_setdiff(pB.originalPlan,pB.taskQueue);
        timeForTask(i) = cost(i) + compute_tour_length(locs(i,1:2),taskLeft,robot,speeds(i));
    else
        timeForTask(i) = cost(i) + norm(task(1:2) - locs(i,1:2))/(speeds(i)+1e-4);
    end
end

[~, assIdx] = min(timeForTask);
newLoc = locs;
newLoc(assIdx,1:2) = task(1:2);
end