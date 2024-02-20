function p = PlotAgents(X,Y,AdjMatrix,radii)
G = graph(AdjMatrix);

p = plot(G,'bo','XData',X,'YData',Y);
p.MarkerSize = 4;
axis([0 100 0 100]);
h = circles(X,Y,radii,'facecolor',[.6 .4 .8]);
% set transparency of circles
alpha(h,.5);

end