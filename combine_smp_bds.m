clc
clear
close all

load('pose_sample_bound_radius-maxp_0.10_1.mat')

R_err_bound = log_R_err_bound;
t_err_bound = log_t_err_bound;


for id = 2:5
    fname = sprintf('pose_sample_bound_radius-maxp_0.10_%d.mat',id);
    load(fname)
    for i = 1:length(R_err_bound)
        R_err_bound{i} = [R_err_bound{i};log_R_err_bound{i}];
        t_err_bound{i} = [t_err_bound{i};log_t_err_bound{i}];
    end
    
end

load('bounds_errors/pose_bound_radius-maxp_0.10.mat')

R_bd_avg = cat(1,log_R_err_bound{:});
R_bd_smp = min(real(cat(2,R_err_bound{:}))',[],2);

diff = (R_bd_avg - R_bd_smp);

figure;
a = cdfplot(R_bd_avg);
a.LineWidth = 3;
hold on
b = cdfplot(R_bd_smp);
b.LineWidth = 3;
legend({'Error bounds of average pose','Minimum error bounds of pose samples'},'FontSize',20)


