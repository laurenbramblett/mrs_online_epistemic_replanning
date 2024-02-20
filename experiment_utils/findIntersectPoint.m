function intersectPoint = findIntersectPoint(locA,locB,taskLocsB,velA,velB,dt,goalReached,maxRange,startTime,currTime,depot,robot)
    radius = 0; taskCount = 1; numTasks = size(taskLocsB,1);
    timePast = 0;
    numTimesInLoop = 0;
    %We assume locA is already in the right location but locB may need to
    %be simulated forward in time. If not, startTime = currTime
    while norm(locA-locB)-radius>=maxRange*2-robot.commError
        distToTaskB = norm(locB-taskLocsB(taskCount,1:2));
        angleToTaskB = atan2(taskLocsB(taskCount,2)-locB(2),taskLocsB(taskCount,1)-locB(1));
        speedB = velB;
        locB = locB + [cos(angleToTaskB), sin(angleToTaskB)].*speedB*dt;
        if startTime + timePast >= currTime
            radius = velA*dt + radius;
        end
        if distToTaskB<goalReached && taskCount < numTasks
            taskCount = taskCount + 1;
        elseif distToTaskB<goalReached && taskCount >= numTasks
            taskLocsB = depot;
            taskCount = 1; numTasks = 1;
        end
        timePast = timePast + dt;
        numTimesInLoop = numTimesInLoop + 1;
        if numTimesInLoop>1e8
            return
        end
    end
    angleToGoal = atan2(locB(2)-locA(2),locB(1)-locA(1));
    intersectPoint = locA + [cos(angleToGoal), sin(angleToGoal)]*radius;
end

% hold on
% scatter(locA(1),locA(2),'g*')
% scatter(locB(1),locB(2),'b*')
% h = rectangle('Position',[locA(1)-radius locA(2)-radius radius*2 radius*2],'Curvature',[1,1]);
% lineDraw = [locB; taskLocsB];
% plot(lineDraw(:,1),lineDraw(:,2),'b--')
% hold off