%% THYMECTOMY - POSTPROCESSING
parentDirectory = string(fileparts(cd));
scene = stlread(parentDirectory+"\scene.stl");
thyme = stlread(parentDirectory+"\thyme.stl");
phrenic = stlread(parentDirectory+"\phrenic.stl");

filename = split(cd,"\"); filename = filename{end};
vf = unity2blender(readtable(string(filename)+"_VFs.csv"));

close all;set(0,'defaultfigurecolor',[1 1 1]); figure;
meshplot = trisurf(scene,'FaceColor','#AAAAAA','EdgeAlpha',0,'FaceAlpha',0.3);
hold on; axis equal; axis off;
thymeplot = trisurf(thyme,'FaceColor','#f5f242','EdgeAlpha',0,'FaceAlpha',0.5);
phrenicplot = trisurf(phrenic,'FaceColor','#632A23','EdgeAlpha',0,'FaceAlpha',0.5);
view(3)
plot3(vf.PositionX,vf.PositionY,vf.PositionZ,'-r','LineWidth',2)
scale=1;
density = 1/20; % Fraction of the rows to be plott
quiver3(...
    vf.PositionX(1:round(1/density):end),...
    vf.PositionY(1:round(1/density):end),...
    vf.PositionZ(1:round(1/density):end),...
    vf.ObstacleAvoidanceForceFieldVF_X0(1:round(1/density):end),...
    vf.ObstacleAvoidanceForceFieldVF_Y0(1:round(1/density):end),...
    vf.ObstacleAvoidanceForceFieldVF_Z0(1:round(1/density):end),...
    scale,'b','filled','LineWidth',1.5);
%%
view([-75.8912,33.9560]);
%% POSTPROCESSING + EVALUATION
figure; 
p = [vf.PositionX,vf.PositionY,vf.PositionZ];
f = [vf.ObstacleAvoidanceForceFieldVF_X0,...
    vf.ObstacleAvoidanceForceFieldVF_Y0,...
    vf.ObstacleAvoidanceForceFieldVF_Z0];
dist  = zeros(size(p));
for i=1:size(vf,1)
    dist(i) = min(sum((p(i,:)-phrenic.Points).^2,2));
end
hold on; box on; grid on;
yyaxis left
dist = sqrt(dist);
plot(dist);
xlabel('Time');
ylabel("Distance From Surface [m]");
yyaxis right
plot(sum(f.^2,2));
ylabel("Magnitude of the Force [N]");