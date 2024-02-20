function [root, best_tour, tour_states, tasksAssigned] = mcts_planning(numIters,...
    robots,newTaskDict,partition,opBots,iter,continue_plan,full_replan)
exploration_param = 500; root = cell(length(opBots),1);
if ~continue_plan && ~full_replan
    for r = 1:length(opBots)
        thisBot = robots(opBots(r));
        numOldTasks = length(thisBot.taskQueue);
        oldTasks = [thisBot.globalTaskLocs(thisBot.taskQueue,:), ...
                    thisBot.taskQueue', zeros(numOldTasks,2)];
        if ~isempty(newTaskDict(partition==opBots(r),:))
            tasksAssigned{r} = [thisBot.pose(1:2) 0 0 0; ...
                                newTaskDict(partition == opBots(r),:);...
                                oldTasks];
            start_city = 1;
            root{r} = Node_EP(start_city, start_city, []);
        
            city_names = 1:size(tasksAssigned{r},1);
            iterations = numIters;  % Adjust as needed
            time = robots(opBots(r)).dt*iter;
            [tour_states{r},best_tour{r}] = MCTS_MTSP_EP(root{r}, city_names, ...
                iterations, exploration_param, tasksAssigned{r},time,robots(opBots(r)));
        else
            tasksAssigned{r} = [thisBot.pose(1:2) 0 0 0; ...
                    oldTasks];
            if ~isempty(oldTasks)
                best_tour{r} = 1:size(tasksAssigned{r},1);
                tour_states{r} = tasksAssigned{r}(:,1:2);
            else
                best_tour{r} = 1;
                tour_states{r} = tasksAssigned{r}(:,1:2);
            end
            start_city = 1;
            root{r} = Node_EP(start_city, start_city, []);
        end

    end
    
elseif full_replan
    for r = 1:length(opBots)
        thisBot = robots(opBots(r));
        tasksAssigned{r} = [thisBot.pose(1:2) 0 0 0; ...
                            newTaskDict(partition == opBots(r),:)];

        start_city = 1;
        root{r} = Node_EP(start_city, start_city, []);
    
        city_names = 1:size(tasksAssigned{r},1);
        iterations = 4000;%numIters;  % Adjust as needed
        time = robots(opBots(r)).dt*iter;
        [tour_states{r},best_tour{r}] = MCTS_MTSP_EP(root{r}, city_names, iterations, exploration_param, tasksAssigned{r},time,robots(opBots(r)));
    end
else
    for r = 1:length(opBots)
        root{r} = robots(opBots(r)).root;
        tasksAssigned{r} = [robots(opBots(r)).pose(1:2) 0 0 0;
                            robots(opBots(r)).tasksAssigned];
        city_names = 1:size(tasksAssigned{r},1);
        iterations = numIters;  % Adjust as needed
        time = robots(opBots(r)).dt*iter;
        [tour_states{r},best_tour{r}] = MCTS_MTSP_EP(root{r}, city_names, iterations, exploration_param, tasksAssigned{r},time,robots(opBots(r)));
    end
end