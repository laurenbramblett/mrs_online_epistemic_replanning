% Main MCTS function
function [tour_states,best_tour] = MCTS_MTSP_EP(root, all_cities, iterations, exploration_param, tasks, time, robot)
    for i = 1:iterations
        exploration_param = exploration_param*0.999;
        leaf = traverse(root, all_cities, exploration_param);
        reward = rollout(leaf, all_cities, tasks, time, robot);
        backpropagate(leaf, reward);
    end

    % Extracting the best tour after MCTS
    best_tour = [root.city];
    while ~isempty(root.children)
        root = root.best_child(0);
        best_tour = [best_tour, root.city];
    end
    [tour_states,~] = compute_tour_reward(best_tour, tasks, time, robot);
end

function leaf = traverse(node, all_cities, exploration_param)
    while ~isempty(node.children) || ~node.is_fully_expanded(length(all_cities))
        if ~node.is_fully_expanded(length(all_cities))
            leaf = expand(node, all_cities);
            return;
        else
            node = node.best_child(exploration_param);
        end
    end
    leaf = node;
end

function child = expand(node, all_cities)
    unvisited_cities = setdiff(all_cities, node.visited);
    new_city = unvisited_cities(randi(length(unvisited_cities)));
    child = Node_EP(new_city, [node.tour, new_city], node);
    node.children{end+1} = child;
    node.visited = unique([node.visited node.tour new_city]);
end

function reward = rollout(node, all_cities,tasks, time, robot)
    remaining_cities = setdiff(all_cities, node.tour);
    complete_tour = [node.tour, datasample(remaining_cities, length(remaining_cities), 'Replace', false)];
    [~,tour_length] = compute_tour_reward(complete_tour, tasks, time, robot);  % you should define this function based on your dataset or distance metric
    reward = -tour_length;  % We use negative because it's a minimization problem
end

function backpropagate(node, reward)
    while ~isempty(node)
        node.visits = node.visits + 1;
        node.value = node.value + reward;
        node = node.parent;
    end
end
