%% TRAINING1 - POSTPROCESSING
parentDirectory = string(fileparts(cd));

scene = stlread(parentDirectory+"\scene.stl");
filename = split(cd,"\"); filename = filename{end};
vf = unity2blender(readtable(string(filename)+"_VFs.csv"));
traj = unity2blender(readtable(string(filename)+"_traj0.csv"));

close all;set(0,'defaultfigurecolor',[1 1 1]); figure;
meshplot = trisurf(scene,'FaceColor','#AAAAAA','EdgeAlpha',0,'FaceAlpha',0.3);
hold on; axis equal;
plot3(traj.X,traj.Y,traj.Z,'g','LineWidth',2);
view(3)
plot3(vf.PositionX,vf.PositionY,vf.PositionZ,'-r','LineWidth',2)
scale=0.3;
density = 1/10; % Fraction of the rows to be plott
quiver3(...
    vf.PositionX(1:round(1/density):end),...
    vf.PositionY(1:round(1/density):end),...
    vf.PositionZ(1:round(1/density):end),...
    vf.TrajectoryGuidanceVF_X0(1:round(1/density):end),...
    vf.TrajectoryGuidanceVF_Y0(1:round(1/density):end),...
    vf.TrajectoryGuidanceVF_Z0(1:round(1/density):end),...
    scale,'b','filled','LineWidth',1.5);
zlim([0,6])
%%
view([-39.8346,6.9290]); zoom(2.5); readytoexport;
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