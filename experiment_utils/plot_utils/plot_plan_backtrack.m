function plot_plan_backtrack(robots_t,taskLoc,obsDist,goalReached,makeVid,filename)
numParticles = size(robots_t{1}(1).particles,2);
numBots = size(robots_t{1},1);
cr = cbrewer2('qual','Set1',numBots);
figure(1)
hold on
robots = robots_t{1}; k = 1;
allLocationsPlot = [robots(k).initPose(1:2,1)',robots(k).globalTaskLocs];
maxX = max(allLocationsPlot(:,1)); minX = min(allLocationsPlot(:,1));
maxY = max(allLocationsPlot); minY = min(allLocationsPlot);
meanTasks = mean(allLocationsPlot,1);
maxX_dir = max(meanTasks(1)-minX,maxX-meanTasks(1));
maxY_dir = max(meanTasks(2)-minY,maxY-meanTasks(2));
max_dir = max(maxX_dir,maxY_dir);
rbt_size = max_dir*2/250;
xlim([meanTasks(1)-max_dir,meanTasks(1)+max_dir])
ylim([meanTasks(2)-max_dir,meanTasks(2)+max_dir])
% scatter(taskLoc(:,1),taskLoc(:,2),'filled','p','MarkerFaceColor', ...
%     [0.4940 0.1840 0.5560],'MarkerEdgeColor','k','SizeData',40)
set(gcf,'color','w')

set(gcf, 'ToolBar', 'none');

agentsPlot = gobjects(1, numBots);
agentPlan = gobjects(1, numBots);
Q = gobjects(numBots, 4);
commLine = gobjects(numBots,numBots);
plotPart = gobjects(numBots,numParticles);
goalScatterH = gobjects(1,1);
goalScatterNH = gobjects(1,1);

goalsObtained = zeros(size(taskLoc,1),1);
for t = 1:size(robots_t,2)-1
    robots = robots_t{t};
    delete(goalScatterH); delete(goalScatterNH)
    if sum(goalsObtained)>0
        H = logical(goalsObtained);
        goalScatterH = scatter(taskLoc(H,1),taskLoc(H,2),'filled','x','MarkerFaceColor', ...
                                'g','MarkerEdgeColor','k','SizeData',40);
    end
    if sum(1-goalsObtained)>0
        NH = logical(1-goalsObtained);
        goalScatterNH = scatter(taskLoc(NH,1),taskLoc(NH,2),'filled','p','MarkerFaceColor', ...
                                [0.4940 0.1840 0.5560],'MarkerEdgeColor','k','SizeData',40);
    end
    
    for k = 1:numBots
        pPlots = robots(k).particles;
        distCheck = double(pdist2(taskLoc,robots(k).pose(1:2))<goalReached+1);
        goalsObtained = min(goalsObtained + distCheck,1);
        for r = 1:numBots
            for p = 1:numParticles
                
                if k == 1
                    delete(plotPart(r,p));
                    plotPart(r,p) = scatter(pPlots(r,p).pose(1),pPlots(r,p).pose(2),...
                        36,cr(r,:),'MarkerEdgeColor','none','MarkerFaceColor',cr(r,:),'MarkerFaceAlpha',0.5);
                end
            end
            delete(commLine(k,r));
            if k~=r && pdist2(robots(k).pose(1:2),robots(r).pose(1:2))<2*obsDist
                line = [robots(k).pose(1:2);robots(r).pose(1:2)];
                commLine(k,r) = plot(line(:,1),line(:,2),'g','LineWidth',2,'LineStyle','--');      
            end
        end
        
        delete(agentsPlot(k));
        delete(Q(k,:));
        Q(k,:) =  RosbotMake(robots(k).pose(1:2),robots(k).pose(3),cr(k,:),rbt_size);
        agentsPlot(k) = scatter(NaN,NaN,36,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor',cr(k,:));
        plan = [robots(k).pose(1:2);robots(k).goalLoc];
%         commPlan = find(robots(k).plan(:,4)==0);
%         for c = 1:length(commPlan)
%             plan(commPlan(c),:) = robots(k).particles(commPlan(c),robots(k).particleGuess(commPlan(c))).pose(1:2);
%         end
        delete(agentPlan(k));
        if robots(k).max_v>0
            agentPlan(k) = plot(plan(:,1),plan(:,2),'LineStyle','--','Color',cr(k,:));
        end
    end
    title(sprintf('Time: $%.2f$s',t*robots(1).dt),'interpreter','latex')
    drawnow
    F(t) = getframe(gcf);
end

if makeVid
    vidfn = sprintf("videos/%s",filename);
    makeVideo2(F,vidfn)
end

end