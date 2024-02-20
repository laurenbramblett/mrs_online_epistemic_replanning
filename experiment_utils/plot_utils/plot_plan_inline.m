%plot_plan_inline
iter = i;
if iter == 1
    numParticles = size(robots(1).particles,2);
    numBots      = size(robots,1);

    cr = cbrewer2('qual','Set1',numBots);
    figure(1); hold on
    set(gcf,'color','w')
    
    allLocationsPlot = [initPose(1:2,1)'; robots(k).globalTaskLocs];
    maxX      = max(allLocationsPlot(:,1)); minX = min(allLocationsPlot(:,1));
    maxY      = max(allLocationsPlot(:,2)); minY = min(allLocationsPlot(:,2));
    meanTasks = mean(allLocationsPlot,1);
    maxX_dir  = max(meanTasks(1)-minX,maxX-meanTasks(1));
    maxY_dir  = max(meanTasks(2)-minY,maxY-meanTasks(2));
    max_dir   = max(maxX_dir,maxY_dir);
    
    xlim([meanTasks(1)-max_dir-1,meanTasks(1)+max_dir+1])
    ylim([meanTasks(2)-max_dir-1,meanTasks(2)+max_dir+1])
    rbt_size = max_dir*2/250;
    % scatter(taskLoc(:,1),taskLoc(:,2),'filled','p','MarkerFaceColor', ...
    %     [0.4940 0.1840 0.5560],'MarkerEdgeColor','k','SizeData',40)
    


    agentsPlot = gobjects(1, numBots);
    agentPlan = gobjects(1, numBots);
    Q = gobjects(numBots, 4);
    simPlot = gobjects(numBots,1);
    commLine = gobjects(numBots,numBots);
    plotPart = gobjects(numBots,numBots,numParticles);
    % goalScatter = gobjects(size(taskLoc,1),1);
    goalScatterNH = gobjects(1,1);
    goalScatterH  = gobjects(1,1);
    replanMarker  = gobjects(numBots,1);

    goalsObtained = zeros(size(taskLoc,1),1);
    txt_offset    = max_dir*4/250;
end

    % robots = robots_t{t};
% delete(goalScatterH); delete(goalScatterNH)
whichDead = sum([robots(:).deadBots],2)>0;
whichTasksLeft = my_setdiff(1:size(taskLoc,1),[robots(:).tasks_done]);
delete(goalScatterH); delete(goalScatterNH)
if sum(goalsObtained)>0
    H = logical(goalsObtained);
    goalScatterH = scatter(taskLoc(H,1),taskLoc(H,2),'filled','x','MarkerFaceColor', ...
                            'g','MarkerEdgeColor','k','SizeData',40);
end
if sum(1-goalsObtained)>0
    whichDead = sum([robots(:).deadBots],2)>0;
    NH = logical(1-goalsObtained);
    taskColors = ones(size(taskLoc,1),3)*0.5;
    for k = 1:numBots
        if ~whichDead(k)
            taskColors(robots(k).taskQueue,:) = repmat(cr(k,:),length(robots(k).taskQueue),1);
        end
    end
    taskPose = taskLoc(NH,:);
    taskColors = taskColors(NH,:);
    goalScatterNH = scatter(taskPose(:,1),taskPose(:,2),40,taskColors,'filled','Marker','pentagram','MarkerEdgeColor','k');
end

for k = 1:numBots
    pPlots = robots(k).particles;
    % distCheck = double(pdist2(taskLoc,robots(k).pose(1:2))<goalReached+1);
    goalsObtained(unique([robots(:).tasks_done])) = 1;
    
    for r = 1:numBots
        for p = 1:numParticles
            
            % if k == 1
            delete(plotPart(k,r,p));
            plotPart(k,r,p) = scatter(pPlots(r,p).pose(1),pPlots(r,p).pose(2),...
                36,cr(r,:),'MarkerEdgeColor','none','MarkerFaceColor',cr(r,:),'MarkerFaceAlpha',0.5);
            % end
        end
        delete(commLine(k,r));
        if k~=r && pdist2(robots(k).pose(1:2),robots(r).pose(1:2))<2*obsDist
            line = [robots(k).pose(1:2);robots(r).pose(1:2)];
            commLine(k,r) = plot(line(:,1),line(:,2),'g','LineWidth',2,'LineStyle','--');      
        end
    end
end

for k = 1:numBots
    delete(agentsPlot(k));
    delete(Q(k,:));
    Q(k,:) =  RosbotMake(robots(k).pose(1:2),robots(k).pose(3),cr(k,:),rbt_size); %1 for 200x200
    agentsPlot(k) = scatter(NaN,NaN,36,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor',cr(k,:));
    plan = [robots(k).pose(1:2);robots(k).goalLoc];
    
%         commPlan = find(robots(k).plan(:,4)==0);
%         for c = 1:length(commPlan)
%             plan(commPlan(c),:) = robots(k).particles(commPlan(c),robots(k).particleGuess(commPlan(c))).pose(1:2);
%         end

    delete(agentPlan(k));
    if robots(k).max_v > 0
        agentPlan(k) = plot(plan(:,1),plan(:,2),'LineStyle','--','Color',cr(k,:));
    end

    delete(replanMarker(k)); 
    if (~isempty(robots(k).commQueue) || ~isempty(robots(k).rzQueue) || ~isempty(my_setdiff(robots(k).taskQueue,robots(k).originalPlan))) && robots(k).max_v>0.1
        replanMarker(k) = text(robots(k).pose(1) - txt_offset,robots(k).pose(2) + txt_offset*3,'R','FontSize',7);
    elseif robots(k).max_v<0.1
        replanMarker(k) = text(robots(k).pose(1) - txt_offset/2, robots(k).pose(2) + txt_offset*3,'D','FontSize',7);
    end
    delete(simPlot(k))
    simPlot(k) = scatter(robots(k).pose_sim(1),robots(k).pose_sim(2),50,'g','x');
end

title(sprintf('Time: $%.2f$s',iter*robots(1).dt),'interpreter','latex')
drawnow
