d = 0:50;
a = 1-exp(-0.5*d);
u = 0.8*(1-exp(-0.1*d));
d = [d,51:60];
a = [a,NaN*ones(1,10)];
u = [u,NaN*ones(1,10)];
d = [d,61:70];

interpreterlatex
a = [a,0.95+0.05*(1-exp(-0.7*d(1:10)))];
u = [u,0.6+0.2*(1-exp(-0.1*d(1:10)))];
close all
wfigure
plot(d,a,'-o',LineWidth=1.2,MarkerSize=2.5);
hold on
plot(d,u,'-o',LineWidth=1.2,MarkerSize=2.5);
plot([d(51),d(62)],[a(51),a(62)],':','Color','#0072BD')
plot([d(51),d(62)],[u(51),u(62)],':','Color','#D95319')
ylim([-0.1;1.1])
xlim([0,75]);
ylabel("Performance");
xlabel("Days of training");
legend("Assisted","Non-Assisted",'Location','south')
grid on
xticklabels(xticks/10);
title("\bf{Expected learning curves}")