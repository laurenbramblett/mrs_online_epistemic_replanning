function time = compute_tour_length(start,tour,robot,speed)
    time = 0;
    current_loc = start;
    for i = 1:length(tour)-1
        nextCityLoc = robot.globalTaskLocs(tour(i),:);
        time = time + norm(nextCityLoc-current_loc)/speed;
        current_loc = nextCityLoc;
    end
end