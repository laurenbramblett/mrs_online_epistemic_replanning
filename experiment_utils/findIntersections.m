% clc; clear; close
% addpath(genpath('utils'))
% %find all crosses in a plan
% numTasks = 10; numAgents = 3;
% tasks = rand(numTasks,2)*30;
% poses = zeros(numAgents,2);
% maxRange = 3;
% vels = ones(numAgents,1);
% [knapsack] = quickGA_mTSP_inline(tasks,poses,vels);
% [timeWindows]=findIntersections_2(knapsack,tasks,poses,vels,maxRange);

function [timeWindows] = findIntersections(knapsack,tasks,poses,vels,maxRange)
numAgents = length(knapsack);
depot = tasks(end,:);
dist = vecnorm(poses-depot,2,2);
task_holds = ones(numAgents,1); dt = 0.1;
connections = cell(numAgents,1);
count = 0;
while any(dist>1)
    for i = 1:numAgents
        if task_holds(i)>length(knapsack{i})
            taskToDo = depot;
        else
            taskToDo = tasks(knapsack{i}(task_holds(i)),:);
        end
        agentDist = pdist2(poses(i,:),taskToDo);
        if agentDist<1
            task_holds(i) = task_holds(i) + 1;
        end
        theta_d = atan2(taskToDo(2)-poses(i,2),...
                        taskToDo(1)-poses(i,1));
        poses(i,1) = poses(i,1) + vels(i)*cos(theta_d)*dt;
        poses(i,2) = poses(i,2) + vels(i)*sin(theta_d)*dt;
    end
    distMat = squareform(pdist(poses(:,1:2)))<2*maxRange;
    bins = conncomp(graph(distMat));
    for j = 1:numAgents
        % if norm(poses(j,1:2)-depot)<maxRange*2
        %     continue
        % end
        whichConns = find(bins(j)==bins);
        whichConns(whichConns == j) = [];
        if length(whichConns)>1
            for k = 1:length(whichConns)
                connections{j} = [connections{j}; mean(poses(bins(j)==bins,:),1),j,whichConns(k),dt*count];
            end
        end
    end
    dist = vecnorm(poses-depot,2,2);
    count = count + 1;
end

timeWindows = [];
for k = 1:numAgents
    rz = connections{k};
    if ~isempty(rz)
        startTime = rz(1,end);
        hold = startTime; startRow = 1;
        sortRz = sortrows(rz,[3 4 5]);
        for i = 2:size(rz,1)
            if abs(sortRz(i,end)-(hold + dt))>1e-4
                finishTime = sortRz(i-1,end);
                avgPos = mean(sortRz(startRow:i-1,1:2),1);
                timeWindows = [timeWindows; avgPos k sortRz(i-1,end-1) startTime finishTime];
                startTime = sortRz(i,end);
                startRow = i;
                hold = startTime;
            else
                hold = hold + dt;
            end
        end
    end
end
end

% Example
% clc; clear; close
% addpath(genpath('utils'))
% %find all crosses in a plan
% numTasks = 10; numAgents = 3;
% tasks = rand(numTasks,2)*30;
% poses = zeros(numAgents,2);
% maxRange = 3;
% vels = ones(numAgents,1);
% [knapsack] = quickGA_mTSP_inline(tasks,poses,vels);
% [timeWindows]=findIntersections_2(knapsack,tasks,poses,vels,maxRange);