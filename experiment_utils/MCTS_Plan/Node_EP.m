classdef Node_EP < handle
    properties
        city
        tour
        visited
        visits = 0
        value = 0
        children = {}
        parent
        state
    end
    
    methods
        function obj = Node_EP(city, tour, parent)
            if nargin > 0
                obj.city = city;
                obj.tour = tour;
                obj.parent = parent;
                obj.visited = unique([city tour]);
            end
        end

        function isFullyExpanded = is_fully_expanded(obj, total_cities)
            isFullyExpanded = length(obj.children) == (total_cities - length(obj.tour));
        end

        function bestChild = best_child(obj, exploration_param)
            ucb_values = zeros(1, length(obj.children));
            for i = 1:length(obj.children)
                child = obj.children{i};
                exploitation = child.value / (child.visits + 1e-10);
                exploration = exploration_param * sqrt(log(obj.visits + 1e-10) / (child.visits + 1e-10));
                ucb_values(i) = exploitation + exploration;
            end
            [~, idx] = max(ucb_values);
            bestChild = obj.children{idx};
        end
    end
end

