function plot_plan_pretty_experiments_sideways(robots_t,taskLoc,obsDist,makeVid,filename,...
    fImg_robots,no_fImg_robots,alphachannel,no_alphachannel,time_start,initPose,colorDataStore,depot)
numParticles = size(robots_t{1}(1).particles,2);
numBots = size(robots_t{1},1);
cr = cbrewer2('qual','Set1',numBots);
figure('Position',[923.8000  236.6000  500.0000  400.0000]);
hold on
robots = robots_t{1}; k = 1;
allLocationsPlot = [initPose(1:2,1)'; robots(k).globalTaskLocs; robots(k).depot];
maxX = max(allLocationsPlot(:,1)); minX = min(allLocationsPlot(:,1));
maxY = max(allLocationsPlot(:,1)); minY = min(allLocationsPlot(:,2));
meanTasks = mean(allLocationsPlot,1);
maxX_dir = max(meanTasks(1)-minX,maxX-meanTasks(1));
maxY_dir = max(meanTasks(2)-minY,maxY-meanTasks(2));
max_dir = max(maxX_dir,maxY_dir);
rbt_size = max_dir*2/110;
txt_offset = max_dir*4/150;
flag_offset = rbt_size/1;
xlim([meanTasks(1)-max_dir-1,meanTasks(1)+max_dir+1])
ylim([meanTasks(2)-max_dir-1,meanTasks(2)+max_dir+1])
% scatter(taskLoc(:,1),taskLoc(:,2),'filled','p','MarkerFaceColor', ...
%     [0.4940 0.1840 0.5560],'MarkerEdgeColor','k','SizeData',40)
set(gcf,'color','w')
set(gcf, 'ToolBar', 'none');

agentsPlot = gobjects(1, numBots);
agentsPlotMark = gobjects(1,numBots);
agentPlan = gobjects(1, numBots);
agentHistory = gobjects(1,numBots);
Q = gobjects(numBots, 3);
commLine = gobjects(numBots,numBots);
plotPart = gobjects(numBots,numBots,numParticles);
goalScatter = gobjects(size(taskLoc,1),1);
replanMarker = gobjects(numBots,1);
startPlot = gobjects(1,1);
plotPlan = gobjects(1,1);
depotPlot = gobjects(1,1);
simPlot = gobjects(numBots,1);
plotNames = {};
plottedTaskLocs = taskLoc(numBots+1:end,:);
initPose = [];

goalsObtained = zeros(size(taskLoc,1),1);
for t = time_start:size(robots_t,2)-1
    robots = robots_t{t};
    whichDead = sum([robots(:).deadBots],2)>0;
    if sum(goalsObtained)>0
        H = logical(goalsObtained);
        whichTasks = find(H);
        notaskPose = taskLoc(whichTasks,:);
        for j = 1:length(whichTasks)
            delete(goalScatter(whichTasks(j)));
            %if whichTasks(j)>numBots %&& whichTasks(j)~=8
                goalScatter(whichTasks(j)) = image('CData',no_fImg_robots{colorDataStore(whichTasks(j))},...
                    'XData',[notaskPose(j,1)-flag_offset*4 notaskPose(j,1)+flag_offset*4],...
                    'YData',[notaskPose(j,2)-flag_offset*4.2 notaskPose(j,2)+flag_offset*4.2],...
                    'AlphaData',no_alphachannel);
            % end
        end
    end
    if sum(1-goalsObtained)>0
        NH = logical(1-goalsObtained);
        whichTasks = find(NH);
        taskColors = zeros(size(taskLoc,1),1) + numBots + 1;
        for k = 1:numBots
            if ~whichDead(k)
                taskColors(robots(k).taskQueue) = repelem(k,length(robots(k).taskQueue));
            end
        end
        taskPose = taskLoc(NH,:);
        taskColors = taskColors(NH);
        for j = 1:size(taskPose,1)
            if ~isequal(depot,taskPose(j,1:2)) %&& whichTasks(j)~=8%&& whichTasks(j)>numBots
                delete(goalScatter(whichTasks(j)));
                goalScatter(whichTasks(j)) = image('CData',fImg_robots{taskColors(j)},...
                    'XData',[taskPose(j,1)-flag_offset*4 taskPose(j,1)+flag_offset*4],...
                    'YData',[taskPose(j,2)-flag_offset*4 taskPose(j,2)+flag_offset*4],...
                    'AlphaData',alphachannel);
            % goalScatter(whichTasks(t)) = scatter(taskLoc(whichTasks(j),1),taskLoc(whichTasks(t),2),'filled','x','MarkerFaceColor', ...
            %                     'g','MarkerEdgeColor','k','SizeData',40);
            end
        end
        
        % goalScatterNH = scatter(taskPose(:,1),taskPose(:,2),40,taskColors,'filled','Marker','diamond','MarkerEdgeColor','k');
    end
    for k = 1:numBots
        pPlots = robots(k).particles;
        scatter(robots(k).pose_sim(1),robots(k).pose_sim(2),30,cr(k,:),'filled','+')
        % distCheck = double(pdist2(taskLoc,robots(k).pose(1:2))<goalReached+1);
        goalsObtained(unique([robots(:).tasks_done])) = 1;
        for r = 1:numBots
            for p = 1:numParticles
                
                % if k == 1
                delete(plotPart(k,r,p));
                plotPart(k,r,p) = scatter(pPlots(r,p).pose(1),pPlots(r,p).pose(2),...
                    220,cr(r,:),'MarkerEdgeColor','none','MarkerFaceColor',cr(r,:),'MarkerFaceAlpha',0.5);
                % end
            end
        end
    end
    for k = 1:numBots
        plan = [robots(k).pose(1:2);robots(k).goalLoc];
        delete(agentPlan(k));
        if robots(k).max_v>0
            agentPlan(k) = plot(plan(:,1),plan(:,2),'LineStyle','-','Color',cr(k,:),'LineWidth',1.2);
        end
        for r = 1:numBots
            delete(commLine(k,r));
            if k~=r && pdist2(robots(k).pose(1:2),robots(r).pose(1:2))<2*obsDist
                line = [robots(k).pose(1:2);robots(r).pose(1:2)];
                commLine(k,r) = plot(line(:,1),line(:,2),'Color',[255,192,0]./255,'LineWidth',2,'LineStyle','-');      
            end
        end
        delete(agentHistory(k))

       
        agentHistory(k) = plot(robots(k).history(:,1),robots(k).history(:,2),'LineStyle',':','Color',[cr(k,:) 0.75],'LineWidth',1.5);
        delete(agentsPlot(k));
        delete(Q(k,:));
        Q(k,:) =  plotUAV(robots(k).pose(1:2),robots(k).pose(3),rbt_size*3,cr(k,:));
        agentsPlot(k) = scatter(NaN,NaN,36,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor',cr(k,:));
        
        
        delete(replanMarker(k)); 
        if (~isempty(robots(k).commQueue) || ~isempty(robots(k).rzQueue) || ~isempty(my_setdiff(robots(k).taskQueue,robots(k).originalPlan))) && robots(k).max_v>0.1
            replanMarker(k) = text(robots(k).pose(1) + txt_offset*4,robots(k).pose(2) - txt_offset/2,'R','FontSize',10);
            set(replanMarker(k),'Rotation',270);
        elseif robots(k).max_v<0.1
            replanMarker(k) = text(robots(k).pose(1) + txt_offset*4, robots(k).pose(2)  - txt_offset/2 ,'D','FontSize',10);
            set(replanMarker(k),'Rotation',270);
        end
    
        % scatter(plan_hold)
        if t == time_start
            delete(agentsPlotMark(k));
            agentsPlotMark(k) = scatter(NaN,NaN,36,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor',cr(k,:));
            plotNames{end+1} = sprintf("Agent %d",k);
            initPose(k,:) = robots(k).pose(1:2);
        end
        delete(simPlot(k))
        % simPlot(k) = scatter(robots(k).pose_sim(1),robots(k).pose_sim(2),50,'g','x');
    end
    if t == time_start
        depots_all = reshape([robots(:).depot]',[],numBots)';
        plotPlan = plot(NaN,NaN,'LineStyle','-','Color',[0.5,0.5,0.5]);
        plotNames{end+1} = "Route";
        startPlot = scatter(initPose(:,1),initPose(:,2),60,'k','^','filled');
        plotNames{end+1} = "Start Depot";
        depotPlot = scatter(depots_all(:,1),depots_all(:,2),60,'k','v','filled');
        plotNames{end+1} = "End Depot";
        beliefsPlot = scatter(NaN,NaN,36,'Marker','o','MarkerEdgeColor','none','MarkerFaceColor',[0.5,0.5,0.5]);
        plotNames{end+1} = "Beliefs";
        % leg = legend(orderIllust, pltNewNames,'Location','bestoutside','NumColumns',3);
        commPlot = plot(NaN,NaN,'Color',[255,192,0]./255,"LineWidth",1.5);
        plotNames{end+1} = "Connected";
        
    end
    % leg = legend([agentsPlotMark,plotPlan,startPlot,depotPlot,beliefsPlot],plotNames,'Location','bestoutside','NumColumns',4);
    % leg = legend([agentsPlotMark,plotPlan,startPlot,depotPlot,beliefsPlot],plotNames,'Location','northoutside','NumColumns',8);
    leg = legend([agentsPlotMark,plotPlan,startPlot,depotPlot,beliefsPlot,commPlot],plotNames,'Location','northoutside','NumColumns',4);

    leg.ItemTokenSize = [6,18];

    title(sprintf('Time: $%.2f$s',t*robots(1).dt),'interpreter','latex')
    drawnow
    % if t == 54 || t == 78 || t == 164
    %     keyboard;
    % end
    F(t) = getframe(gcf);
end

if makeVid
    vidfn = sprintf("videos/%s",filename);
    makeVideo2(F,vidfn)
end

end