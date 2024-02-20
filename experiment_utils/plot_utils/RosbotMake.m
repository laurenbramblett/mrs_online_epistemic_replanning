%% Prepare the Rosbot Points Before Plotting
function Q = RosbotMake(p,heading_ang,lightbox,scale)
% Choose the scaling factor for plotting Rosbot
params.RosbotScale = scale; %0.3 for otherplots 0.1 for exp

params.redBox  = params.RosbotScale*[ -2 -2 2 2 ; 1.6 -1.6 -1.6 1.6];
params.leftTires  = params.RosbotScale*[ .35 .35 2 2 .35 -.35 -2 -2 -.35 -.35 ; 1.6 1.9 1.9 1.6 1.6 1.6 1.6 1.9 1.9 1.6];
params.rightTires = params.RosbotScale*[ .35 .35 2 2 .35 -.35 -2 -2 -.35 -.35 ; -1.6 -1.9 -1.9 -1.6 -1.6 -1.6 -1.6 -1.9 -1.9 -1.6];


r = .6;
GPSx = params.RosbotScale*r*cos( 0:(pi/10):(2*pi) ) + params.RosbotScale*.8;
GPSy = params.RosbotScale*r*sin( 0:(pi/10):(2*pi) );
params.UGV_GPS    = [ GPSx ; GPSy];
params.lightBar   = params.RosbotScale*[ -.6 -.6 -1.25 -1.25 ; 1.2 -1.2 -1.2 1.2];


%% Draw the points
% Rotation Matrix (with heding angle at time k
R = [ cos(heading_ang ) -sin(heading_ang) ; sin(heading_ang) cos(heading_ang) ];

% Current robot position: [x, y]^T coordinates at time k
Rosbot_pos = p';
Q = gobjects(1, 4);
% Plot Points as patches
yB = R*params.redBox + Rosbot_pos;
Q(1) = patch(yB(1,:),yB(2,:),lightbox,'FaceColor',lightbox,'EdgeColor','k','LineWidth',.6);
lT = R*params.leftTires + Rosbot_pos;
Q(2) = patch(lT(1,:),lT(2,:),'k','FaceColor','k','EdgeColor','k','LineWidth',.8);
rT = R*params.rightTires + Rosbot_pos;
Q(3) = patch(rT(1,:),rT(2,:),'k','FaceColor','k','EdgeColor','k','LineWidth',.8);
Gps = R*params.UGV_GPS + Rosbot_pos;
Q(4) = patch(Gps(1,:),Gps(2,:),'k','FaceColor','k','EdgeColor','k','LineWidth',.2);
% lB = R*params.lightBar + Rosbot_pos;
% Q.Rosbot1_P5 = patch(lB(1,:),lB(2,:),lightbox,'FaceColor',lightbox,'EdgeColor','k','LineWidth',1.2);
end

%%%% Then every iteration delete the above fields within "Q"
        
        