function [pose,force] = move_robot_forces_experiment(robot, map, goal, obsRho)

    numParticles = size(robot.particles, 2);
    dt           = robot.dt;
    pose         = robot.pose; 
    max_v        = robot.max_v + robot.max_v*0.1;
    
    % ------------------- %
    %% Obs Force
    % ------------------- %
    % obsIdx       = find(robot.M0>0.5);
    % [r_o,c_o]    = ind2sub(map.size,obsIdx);
    % obs          = [c_o,r_o];
    % if ~isempty(obs)
    %     obsDist = pdist2(pose(1:2),obs)';
    % else
    %     obsDist = [];
    % end
    % c_obs = obs(obsDist<obsRho,:); obsDist = obsDist(obsDist<obsRho,:);
    % if ~isempty(c_obs)
    %     [minVal,minIdx] = min(obsDist);
    %     dirObs = c_obs(minIdx,:)-pose(1:2);
    %     if minVal>1.5
    %         obsSpring = -(dirObs)./minVal*(1/1.5-1/minVal)*(1/minVal.^2);
    %     else
    %         obsSpring = -sign(dirObs).*[1000,1000];
    %     end
    % else
    %     obsSpring = [0,0]; 
    % end

    % ------------------- %
    %% Goal Force
    goalDist = pdist2(pose(1:2),goal(1:2));
    if goalDist>0
        goalNorm = (goal(1:2)-pose(1:2))./goalDist;
    else
        goalNorm = [0,0];
    end
    
    goalSpring = goalDist*goalNorm;
    if sqrt(sum(goalSpring.^2))>max_v
        goalSpring = goalSpring*max_v/sqrt(sum(goalSpring.^2));
    end


    % ------------------- %
    %% Combined Force
    % ------------------- %
    v = 1/dt*goalSpring; %+ 6*obsSpring;% + 6/dt*otherSpring;

    if sqrt(sum(v.^2)) > max_v 
        v = v*max_v/sqrt(sum(v.^2));
    end 
    
    if norm(v)<0.1
        v = [0,0];
    end
    force = v;
    
    theta_d = wrapToPi(atan2(v(2),v(1))-pose(3));
    % theta_d = sign(theta_d)*min(pi,abs(theta_d));
    vel = norm(v)*cos(theta_d);
    pose(3) = wrapToPi(pose(3)+theta_d);
    pose(1) = pose(1) + (vel*cos(pose(3))+normrnd(0,sqrt(0)))*dt;
    pose(2) = pose(2) + (vel*sin(pose(3))+normrnd(0,sqrt(0)))*dt;
    
end