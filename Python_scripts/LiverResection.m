%% SCENE SETUP
scene = stlread("C:\Users\alber\Desktop\LiverResection.stl");
d = readtable("C:\Users\alber\Desktop\LiverResection_26\LiverResection_26_VFs.csv");
o = readtable("C:\Users\alber\Desktop\LiverResection_26\LiverResection_26_surfguide0.csv");
o2 = stlread("C:\Users\alber\Desktop\cut.stl");
% t = readtable("C:\Users\alber\Desktop\SandBox_2\SandBox_2_traj0.csv");
% t2 = readtable("C:\Users\alber\Desktop\SandBox_2\SandBox_2_traj1.csv");
% o = readtable("C:\Users\alber\Desktop\SandBox_2\SandBox_2_obst0.csv");
% o2 = readtable("C:\Users\alber\Desktop\SandBox_2\SandBox_2_obst1.csv");
close
figure; alpha = 0.2;
meshplot = trisurf(scene,'FaceColor','#AAAAAA','EdgeAlpha',0,'FaceAlpha',0.3);
hold on
surfplot = trisurf(o2,'FaceColor','#00FF00','EdgeAlpha',0,'FaceAlpha',0.1);
axis equal
%% MAKE SCENE EXPORTABLE
meshplot.EdgeAlpha = 0.02;
meshplot.FaceAlpha = 0;
surfplot.FaceAlpha = 0.5;
%%
hold on
xlabel('X');ylabel('Y');zlabel('Z');
view(3)
plot3(-d.PositionX,-d.PositionZ,d.PositionY,'-r','LineWidth',2)
% plot3(-t.X,-t.Z,t.Y,'-g','LineWidth',2)
% plot3(-t2.X,-t2.Z,t2.Y,'-m','LineWidth',2)
% scatter3(-o.X,-o.Z,o.Y,'g','filled','MarkerFaceAlpha',0.1)
% scatter3(-o2.X,-o2.Z,o2.Y,'g','filled')
s=10;
quiver3(-d.PositionX(1:5:end),-d.PositionZ(1:5:end), d.PositionY(1:5:end),...
    -d.SurfaceGuidanceVF_X0(1:5:end),...
    -d.SurfaceGuidanceVF_Z0(1:5:end),...
    d.SurfaceGuidanceVF_Y0(1:5:end),s,'b','filled');
% % quiver3(-d.PositionX,-d.PositionZ, d.PositionY,...
% %     -d.TrajectoryGuidanceVF_X1,...
% %     -d.TrajectoryGuidanceVF_Z1,...
% %     d.TrajectoryGuidanceVF_Y1,s,'c','filled');
% quiver3(-d.PositionX,-d.PositionZ, d.PositionY,...
%     -d.ObstacleAvoidanceForceFieldVF_X0,...
%     -d.ObstacleAvoidanceForceFieldVF_Z0,...
%     d.ObstacleAvoidanceForceFieldVF_Y0,s,'y','filled');
% quiver3(-d.PositionX,-d.PositionZ, d.PositionY,...
%     -d.ObstacleAvoidanceForceFieldVF_X1,...
%     -d.ObstacleAvoidanceForceFieldVF_Z1,...
%     d.ObstacleAvoidanceForceFieldVF_Y1,s,'g','filled');
axis equal
% legend("Scene","Trajectory","Force","Location","northeast")