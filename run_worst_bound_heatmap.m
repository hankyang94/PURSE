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

% fname = sprintf("../6D_Pose/pose_uncertain_ellinf_%s_%.2f.mat",score_type,epsilon);
fname = sprintf("/Users/hankyang/Downloads/pose_uncertain_ellinf_%s_%.2f.mat",score_type,epsilon);
if do_frcnn
    fname = sprintf("../6D_Pose/pose_uncertain_ellinf_%s_%.2f_frcnn.mat",score_type,epsilon);
end
load(fname)

n_objs = length(A);
log_R_err_bound = {};
log_t_err_bound = {};
log_R_gap       = {};
log_t_gap       = {};

for objidx = 1:n_objs
    n_samples       = size(A{objidx},1);
    obj_A           = A{objidx};
    obj_B           = B{objidx};
    obj_pose_avg    = pose_avg{objidx};
    obj_R_avg_smp_ucrt = R_avg_smp_ucrt{objidx};
    obj_t_avg_smp_ucrt = t_avg_smp_ucrt{objidx};

    obj_R_err_bound = zeros(n_samples,1);
    obj_t_err_bound = zeros(n_samples,1);
    obj_R_gap       = zeros(n_samples,1);
    obj_t_gap       = zeros(n_samples,1);

    for idx = 1:n_samples
        T = squeeze(obj_pose_avg(idx,:,:));
        R_avg = T(:,1:3);
        t_avg = T(:,end);
        A_c = squeeze(obj_A(idx,:,:,:));
        B_c = squeeze(obj_B(idx,:,:));
        % Compute maximum rotation distance
        [R_max,~,info] = compute_max_rotation_distance(R_avg,A_c,B_c);
        if ~isempty(R_max) && (info.gap < 1e-6)
            R_err_max = getAngularError(R_max,R_avg);
            fprintf("idx: %d, R_err_max: %3.2e, R_err_max_smp: %3.2e.\n",idx,R_err_max,obj_R_avg_smp_ucrt(idx));
            obj_R_err_bound(idx) = R_err_max;
        else
            R_err_bound = rad2deg(acos(1 - info.f_sdp/4));
            fprintf("idx: %d, R_err_bound: %3.2e, R_err_max_smp: %3.2e.\n",idx,R_err_bound,obj_R_avg_smp_ucrt(idx));
            obj_R_err_bound(idx) = R_err_bound;
        end
        obj_R_gap(idx) = info.gap;
        
        % Compute maximum translation distance
        [~,t_max,info] = compute_max_translation_distance(t_avg,A_c,B_c);
        if ~isempty(t_max) && (info.gap < 1e-6)
            t_err_max = getTranslationError(t_max,t_avg);
            fprintf("idx: %d, t_err_max: %3.2e, t_err_max_smp: %3.2e.\n",idx,t_err_max,obj_t_avg_smp_ucrt(idx));
            obj_t_err_bound(idx) = t_err_max;
        else
            t_err_bound = sqrt(info.f_sdp);
            fprintf("idx: %d, t_err_bound: %3.2e, t_err_max_smp: %3.2e.\n",idx,t_err_bound,obj_t_avg_smp_ucrt(idx));
            obj_t_err_bound(idx) = t_err_bound;
        end
        obj_t_gap(idx) = info.gap;
    end
    log_R_err_bound = [log_R_err_bound;{obj_R_err_bound}];
    log_t_err_bound = [log_t_err_bound;{obj_t_err_bound}];
    log_R_gap       = [log_R_gap;{obj_R_gap}];
    log_t_gap       = [log_t_gap;{obj_t_gap}];
end

matname = sprintf("pose_bound_%s_%.2f.mat",score_type,epsilon);
if do_frcnn
    matname = sprintf("pose_bound_%s_%.2f_frcnn.mat",score_type,epsilon);
end
save(matname,'log_R_err_bound','log_t_err_bound','log_R_gap','log_t_gap');


