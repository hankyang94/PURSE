clc
clear 
close all

score_type = 'radius-cov-topk';
epsilon = 0.4;
do_frcnn = true;

fname1 = sprintf("bounds_errors/pose_bound_%s_%.2f.mat",score_type,epsilon);
if do_frcnn
    fname1 = sprintf("bounds_errors/pose_bound_%s_%.2f_frcnn.mat",score_type,epsilon);
end
load(fname1)
fname2 = sprintf("bounds_errors/pose_avg_err_%s_%.2f.mat",score_type,epsilon);
if do_frcnn
    fname2 = sprintf("bounds_errors/pose_avg_err_%s_%.2f_frcnn.mat",score_type,epsilon);
end
load(fname2)

R_errs = cat(2,R_avg_err{:})';
t_errs = cat(2,t_avg_err{:})';
R_bds = cat(1,log_R_err_bound{:});
t_bds = cat(1,log_t_err_bound{:});
coverage = cat(2,pose_coverage{:})';
R_gap = cat(1,log_R_gap{:});

scattersize = 80;
scatteralpha = 0.1;
labelsize = 40;
ticksize = 15;
legendsize = 18;

%% Plot rotation figure
figure('Position', [100 100 450 450]);
scatter(R_bds(~coverage),R_errs(~coverage),scattersize,'square','filled',...
    'MarkerFaceColor','red','MarkerEdgeColor','red',...
    'MarkerFaceAlpha',scatteralpha,'MarkerEdgeAlpha',scatteralpha);
hold on
scatter(R_bds(coverage),R_errs(coverage),scattersize,'filled',...
    'MarkerFaceColor','blue','MarkerEdgeColor','blue',...
    'MarkerFaceAlpha',scatteralpha,'MarkerEdgeAlpha',scatteralpha);
axis equal
xlim([0,180]);
ylim([0,180]);
xlabel('Worst-case rotation error bound (deg)','FontSize',labelsize)
ylabel('Rotation error b/t avg pose and gt pose (deg)','FontSize',labelsize)
plot([0,180],[0,180],'cyan','LineWidth',3,'HandleVisibility','off');
legend({'Uncovered','Covered'},'FontSize',legendsize,'Location','north','NumColumns',2)
ax = gca;
ax.FontSize = ticksize;

%% Plot translation figure
t_max = 5;
% t_bds(t_bds > t_max) = t_max;
figure('Position', [100 100 450 450]);
scatter(t_bds(~coverage),t_errs(~coverage),scattersize,'square','filled',...
    'MarkerFaceColor','red','MarkerEdgeColor','red',...
    'MarkerFaceAlpha',scatteralpha,'MarkerEdgeAlpha',scatteralpha);
hold on
scatter(t_bds(coverage),t_errs(coverage),scattersize,'filled',...
    'MarkerFaceColor','blue','MarkerEdgeColor','blue',...
    'MarkerFaceAlpha',scatteralpha,'MarkerEdgeAlpha',scatteralpha);
axis equal
xlim([0,t_max]);
ylim([0,t_max]);
xlabel('Worst-case translation error bound','FontSize',labelsize)
ylabel('Translation error b/t avg pose and gt pose','FontSize',labelsize)
plot([0,t_max],[0,t_max],'cyan','LineWidth',3,'HandleVisibility','off');
legend({'Uncovered','Covered'},'FontSize',legendsize,'Location','north','NumColumns',2)
ax = gca;
ax.FontSize = ticksize;



