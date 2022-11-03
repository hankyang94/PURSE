clc
clear 
close all

score_type = 'radius-maxp';
epsilon = 0.4;

fname1 = sprintf("bounds_errors/pose_bound_%s_%.2f.mat",score_type,epsilon);
load(fname1)
fname2 = sprintf("bounds_errors/pose_avg_err_%s_%.2f.mat",score_type,epsilon);
load(fname2)

R_errs = cat(2,R_avg_err{:})';
t_errs = cat(2,t_avg_err{:})';
R_bds = cat(1,log_R_err_bound{:});
t_bds = cat(1,log_t_err_bound{:});
coverage = cat(2,pose_coverage{:})';
R_gap = cat(1,log_R_gap{:});

% Replace wrong results
fname3 = sprintf("abnormal/wrong_ids_%s_%.2f.mat",score_type,epsilon);
load(fname3)
R_bds(abnormal) = correct_R_err_bound;
t_bds(abnormal) = correct_t_err_bound;

scattersize = 80;
scatteralpha = 0.1;
labelsize = 30;
ticksize = 15;
legendsize = 18;

%% Plot rotation figure
figure('Position', [100 100 500 500]);
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
xlabel('Rotation error b/t avg pose and PURSE','FontSize',labelsize)
ylabel('Rotation error b/t avg pose and gt pose','FontSize',labelsize)
plot([0,180],[0,180],'cyan','LineWidth',3,'HandleVisibility','off');
legend({'Uncovered','Covered'},'FontSize',legendsize,'Location','northoutside','NumColumns',2)
ax = gca;
ax.FontSize = ticksize;

%% Plot translation figure
t_max = 4;
t_bds(t_bds > t_max) = t_max;
figure('Position', [100 100 500 500]);
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
xlabel('Translation error b/t avg pose and PURSE','FontSize',labelsize)
ylabel('Translation error b/t avg pose and gt pose','FontSize',labelsize)
plot([0,t_max],[0,t_max],'cyan','LineWidth',3,'HandleVisibility','off');
legend({'Uncovered','Covered'},'FontSize',legendsize,'Location','northoutside','NumColumns',2)
ax = gca;
ax.FontSize = ticksize;



