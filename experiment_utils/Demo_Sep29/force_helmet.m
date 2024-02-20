function [forceX,forceY] = force_helmet(pos,goal,xVel,yVel,k,helmet)
    kg = 0.6; damp = 0.3; kn = 1; distToNeigh = 0.75;%kg = 0.01, damp = 0.05, kn = 0.03, distToNeigh = 1 works for 2 robots
    max_a = 0.2;
    %Obs Force
    oForceX = 0; oForceY = 0;
    %Goal Force
    goalDist = vecnorm(pos(k,:)-goal,2,2);
    goalNorm = (goal-pos(k,:))./goalDist;
    % if goalDist<0.3; goalNorm = [0,0]; end
    gForceX = kg*goalDist*goalNorm(1);
    gForceY = kg*goalDist*goalNorm(2);
    
    %Neighbor Repulsive Force
    otherBots = setdiff(1:size(pos,1),k);
    whichBots = vecnorm(pos(k~=1:end,1:2)-pos(k,1:2),2,2)<distToNeigh;
    whichBots = otherBots(whichBots);
    if ~isempty(whichBots)
        otherPos = pos(whichBots,:);
        nDist = vecnorm(pos(k,:)-otherPos,2,2);
        nNorm = (otherPos-pos(k,:))./nDist;
        nForceX = -sum(kn.*(nDist).*nNorm(:,1));
        nForceY = -sum(kn.*(nDist).*nNorm(:,2));
    else
        nForceX = 0; nForceY = 0;
    end

    if norm(helmet(1:2)-pos(k,:))<distToNeigh*3/2
        hDist = norm(helmet(1:2)-pos(k,:));
        hNorm = (helmet(1:2)-pos(k,:))./hDist;
        hForceX = -sum(kn.*(hDist).*hNorm(:,1));
        hForceY = -sum(kn.*(hDist).*hNorm(:,2));
    else
        hForceX = 0; hForceY = 0;
    end
    
    forceX = nForceX + oForceX + hForceX + gForceX - damp*xVel;
    forceY = nForceY + oForceY + hForceY + gForceY - damp*yVel;
    if norm([forceX,forceY])>max_a
        comps = [forceX,forceY].*max_a/norm([forceX,forceY]);
        forceX = comps(1); forceY = comps(2);
    end
    if norm([forceX,forceY])<0.1
        forceX = 0;
        forceY = 0;
    end

end