function graph = gabriel_graph(points)
    n = size(points, 1);
    graph = [];

    for i = 1:n
        for j = i+1:n
            p = points(i, :);
            q = points(j, :);
            mid_point = (p + q) / 2;
            radius = norm(p - q) / 2;
            inside = false; 

            for k = 1:n
                if k ~= i && k ~= j
                    s = points(k, :);
                    if norm(mid_point - s) < radius
                        inside = false;%%Set this back to true if actually doing
                        break;
                    end
                end
            end

            if ~inside
                graph = [graph; i,j];  % Append the edge to the graph
            end
        end
    end
end

