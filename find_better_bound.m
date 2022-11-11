clc
clear
close all

relaxpath   = "../CertifiablyRobustPerception";
mosekpath   = '../mosek';
addpath(genpath(relaxpath))
addpath(genpath(mosekpath))

score_type  = "radius-maxp";
epsilon     = 0.1;
do_frcnn    = false;

fname = sprintf("/Users/hankyang/Downloads/samples_pose_uncertain_ellinf_%s_%.2f.mat",score_type,epsilon);
% fname = sprintf("../6D_Pose/samples_pose_uncertain_ellinf_%s_%.2f.mat",score_type,epsilon);
% if do_frcnn
%     fname = sprintf("../6D_Pose/samples_pose_uncertain_ellinf_%s_%.2f_frcnn.mat",score_type,epsilon);
% end
load(fname)

n_objs = length(A);
log_R_err_bound = {};
log_t_err_bound = {};

n_Rs = [];
n_ts = [];
for objidx = 1:n_objs
    n_samples       = size(A{objidx},1);
    obj_A           = A{objidx};
    obj_B           = B{objidx};
    obj_R_samples    = R_samples{objidx};
    obj_t_samples    = t_samples{objidx};

    obj_R_bounds = {};
    obj_t_bounds = {};

    for idx = 1:n_samples
        
        n_R = size(obj_R_samples{idx},1);
        n_t = size(obj_t_samples{idx},1);
      
        A_c = squeeze(obj_A(idx,:,:,:));
        B_c = squeeze(obj_B(idx,:,:));
        
        n_Rs = [n_Rs;n_R];
        n_ts = [n_ts;n_t];
    end
    
end

% matname = sprintf("pose_bound_%s_%.2f.mat",score_type,epsilon);
% if do_frcnn
%     matname = sprintf("pose_bound_%s_%.2f_frcnn.mat",score_type,epsilon);
% end
% save(matname,'log_R_err_bound','log_t_err_bound','log_R_gap','log_t_gap');

function [R_bd,t_bd] = compute_bound_one_instance(A_c,B_c,R_avg,t_avg)
[R_max,~,info] = compute_max_rotation_distance(R_avg,A_c,B_c);
if ~isempty(R_max) && (info.gap < 1e-6)
    R_err_max = getAngularError(R_max,R_avg);
    fprintf("idx: %d, R_err_max: %3.2e, R_err_max_smp: %3.2e.\n",idx,R_err_max,obj_R_avg_smp_ucrt(idx));
    R_bd = R_err_max;
else
    R_err_bound = rad2deg(acos(1 - info.f_sdp/4));
    fprintf("idx: %d, R_err_bound: %3.2e, R_err_max_smp: %3.2e.\n",idx,R_err_bound,obj_R_avg_smp_ucrt(idx));
    R_bd = R_err_bound;
end

[~,t_max,info] = compute_max_translation_distance(t_avg,A_c,B_c);
if ~isempty(t_max) && (info.gap < 1e-6)
    t_err_max = getTranslationError(t_max,t_avg);
    fprintf("idx: %d, t_err_max: %3.2e, t_err_max_smp: %3.2e.\n",idx,t_err_max,obj_t_avg_smp_ucrt(idx));
    t_bd = t_err_max;
else
    t_err_bound = sqrt(info.f_sdp);
    fprintf("idx: %d, t_err_bound: %3.2e, t_err_max_smp: %3.2e.\n",idx,t_err_bound,obj_t_avg_smp_ucrt(idx));
    t_bd = t_err_bound;
end
end
