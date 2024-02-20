function agentPlot = plotUGV(center,rot,scale,color)
RosbotMake(center,rot,color,scale)
agentPlot = scatter(NaN,NaN,36,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor',color);
end