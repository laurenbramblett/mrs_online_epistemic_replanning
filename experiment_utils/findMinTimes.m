function [botsComm,minTimes,rzLoc] = findMinTimes(timeWindows,currentTime,botsCovered)
    if ~isempty(timeWindows)
        windowMins = timeWindows(timeWindows(:,5)>currentTime,:);
        commsIdx = any(all(bsxfun(@eq,reshape(botsCovered.',1,1,[]),windowMins(:,3)),2),3);
        botsComm = setdiff(unique(windowMins(commsIdx,4)),botsCovered);
        rzLoc = zeros(length(botsComm),2);
        minTimes = zeros(length(botsComm),1); 
    else
        botsComm = [];
        minTimes = [];
        rzLoc = [];
    end
    for i = 1:length(botsComm)
        whichCommIdx = any(all(bsxfun(@eq,botsComm(i),windowMins(:,3)),2),3);
        subsetComm = windowMins(whichCommIdx,:);
        [minTimes(i),minIdx] = min(subsetComm(:,5));
        rzLoc(i,:) = subsetComm(minIdx,1:2);
    end
end