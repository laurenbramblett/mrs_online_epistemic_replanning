function agentPlot = plotUAV(center,rot,wspan,color)
pt1 =  center + wspan.*[cos(rot+pi/4) sin(rot+pi/4)];
pt2 =  center - wspan.*[cos(rot+pi/4) sin(rot+pi/4)];
pt3 =  center - wspan.*[cos(rot-pi/4) sin(rot-pi/4)];
pt4 =  center + wspan.*[cos(rot-pi/4) sin(rot-pi/4)];

body = [pt1;pt2;pt3;pt4];

agentPlot(1) = plot(body(1:2,1),body(1:2,2),'Color','k','LineWidth',2);
agentPlot(2) = plot(body(3:4,1),body(3:4,2),'Color','k','LineWidth',2);
agentPlot(3) = scatter(body(:,1),body(:,2),30,color,'filled','MarkerFaceAlpha',0.4);
% agentPlot = scatter(NaN,NaN,36,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor',color);
end