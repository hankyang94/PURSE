clc
clear 
close all

score_type = 'radius-cov-topk';
epsilon = 0.4;

fname1 = sprintf("../bounds_errors/pose_bound_%s_%.2f.mat",score_type,epsilon);
load(fname1)
fname2 = sprintf("../bounds_errors/pose_avg_err_%s_%.2f.mat",score_type,epsilon);
load(fname2)

R_errs = cat(2,R_avg_err{:})';
t_errs = cat(2,t_avg_err{:})';
R_bds = cat(1,log_R_err_bound{:});
t_bds = cat(1,log_t_err_bound{:});
coverage = cat(2,pose_coverage{:})';
R_gap = cat(1,log_R_gap{:});
obj_id = {};
smp_id = {};
for i = 1:length(pose_coverage)
    n_samples = length(pose_coverage{i});
    obj_id = [obj_id;{i*ones(n_samples,1)}];
    smp_id = [smp_id;{(1:n_samples)'}];
end
obj_id = cat(1,obj_id{:});
smp_id = cat(1,smp_id{:});

abnormal1 = coverage & (R_errs >= R_bds);
abnormal2 = coverage & (t_errs >= t_bds);
abnormal = abnormal1 | abnormal2;

abnormal_obj_id = obj_id(abnormal);
abnormal_smp_id = smp_id(abnormal);

fname = sprintf("wrong_ids_%s_%.2f.mat",score_type,epsilon);
save(fname,'abnormal','abnormal_obj_id','abnormal_smp_id');