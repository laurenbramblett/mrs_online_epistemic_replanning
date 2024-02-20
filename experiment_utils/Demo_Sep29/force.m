function [forceX,forceY] = force(pos,goal,xVel,yVel,edges,k)
    kg = 0.01; damp = 0.05; kn = 0.03; distToNeigh = 1.5;%kg = 0.01, damp = 0.05, kn = 0.03, distToNeigh = 1 works for 2 robots
    %Obs Force
    oForceX = 0; oForceY = 0;
    %Goal Force
    goalDist = vecnorm(pos(k,:)-goal,2,2);
    goalNorm = (goal-pos(k,:))./goalDist;
    if goalDist<0.3; goalNorm = [0,0]; end
    gForceX = kg*goalDist*goalNorm(1);
    gForceY = kg*goalDist*goalNorm(2);
    
    %Neighbor Force
    if ~isempty(edges)
        conBots = edges(edges(:,1)==k,2);
        conBots = [conBots edges(edges(:,2)==k,1)];
        otherPos = pos(conBots,:);
        nDist = vecnorm(pos(k,:)-otherPos,2,2);
        nNorm = (otherPos-pos(k,:))./nDist;
        nForceX = sum(kn.*(nDist-distToNeigh).*nNorm(:,1));
        nForceY = sum(kn.*(nDist-distToNeigh).*nNorm(:,2));
    else
        nForceX = 0; nForceY = 0;
    end
    
    forceX = nForceX + oForceX + gForceX - damp*xVel;
    forceY = nForceY + oForceY + gForceY - damp*yVel;
    forceX = sign(forceX)*min(abs(forceX),0.2);
    forceY = sign(forceY)*min(abs(forceY),0.2);
    
end