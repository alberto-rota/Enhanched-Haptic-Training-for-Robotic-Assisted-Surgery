%% LIVER RESECTION - POSTPROCESSING
parentDirectory = string(fileparts(cd));
scene = stlread(parentDirectory+"\scene.stl");
surface = stlread(parentDirectory+"\surface.stl");
liver = stlread(parentDirectory+"\liver.stl");

filename = split(cd,"\"); filename = filename{end};
vf = unity2blender(readtable(string(filename)+"_VFs.csv"));

close all; set(0,'defaultfigurecolor',[1 1 1]); figure;
meshplot = trisurf(scene,'FaceColor','#AAAAAA','EdgeAlpha',0,'FaceAlpha',0.3);
hold on; axis equal; axis off;
surfplot = trisurf(surface,'FaceColor','#00FF00','EdgeAlpha',0,'FaceAlpha',0.1);
liverplot = trisurf(liver,'FaceColor','#632A23','EdgeAlpha',0,'FaceAlpha',0.1);
view(3)
plot3(vf.PositionX,vf.PositionY,vf.PositionZ,'-r','LineWidth',2)
scale=10;
density = 1/20; % Fraction of the rows to be plott
quiver3(...
    vf.PositionX(1:round(1/density):end),...
    vf.PositionY(1:round(1/density):end),...
    vf.PositionZ(1:round(1/density):end),...
    vf.SurfaceGuidanceVF_X0(1:round(1/density):end),...
    vf.SurfaceGuidanceVF_Y0(1:round(1/density):end),...
    vf.SurfaceGuidanceVF_Z0(1:round(1/density):end),...
    scale,'b','filled','LineWidth',1.5);
%% POSTPROCESSING + EVALUATION
figure; 
p = [vf.PositionX,vf.PositionY,vf.PositionZ];
f = [vf.SurfaceGuidanceVF_X0,vf.SurfaceGuidanceVF_Y0,vf.SurfaceGuidanceVF_Z0];
dist  = zeros(size(p));
for i=1:size(vf,1)
    dist(i) = min(sum((p(i,:)-surface.Points).^2,2));
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