clc
clear
close all

relaxpath   = "../../CertifiablyRobustPerception";
mosekpath   = '../../mosek';
addpath(genpath(relaxpath))
addpath(genpath(mosekpath))
addpath(genpath('../'))

score_type  = "radius-cov-topk";
epsilon     = 0.1;
do_frcnn    = false;

fname = sprintf("/Users/hankyang/Downloads/pose_uncertain_ellinf_%s_%.2f.mat",score_type,epsilon);
if do_frcnn
    fname = sprintf("/Users/hankyang/Downloads/pose_uncertain_ellinf_%s_%.2f_frcnn.mat",score_type,epsilon);
end
load(fname)

fname_wrong = sprintf("wrong_ids_%s_%.2f.mat",score_type,epsilon);
load(fname_wrong)

n_ids = length(abnormal_obj_id);
correct_R_err_bound = zeros(n_ids,1);
correct_t_err_bound = zeros(n_ids,1);

for id = 1:n_ids
    T = squeeze(pose_avg{abnormal_obj_id(id)}(abnormal_smp_id(id),:,:));
    R_avg = T(:,1:3);
    t_avg = T(:,end);
    A_c = squeeze(A{abnormal_obj_id(id)}(abnormal_smp_id(id),:,:,:));
    B_c = squeeze(B{abnormal_obj_id(id)}(abnormal_smp_id(id),:,:));
    
    % Compute maximum rotation distance
    [R_max,~,info] = compute_max_rotation_distance(R_avg,A_c,B_c);
    if ~isempty(R_max) && (info.gap < 1e-6)
        R_err_max = getAngularError(R_max,R_avg);
        fprintf("idx: %d, R_err_max: %3.2e.\n",id,R_err_max);
        correct_R_err_bound(id) = R_err_max;
    else
        R_err_bound = rad2deg(acos(1 - info.f_sdp/4));
        fprintf("idx: %d, R_err_bound: %3.2e.\n",id,R_err_bound);
        correct_R_err_bound(id) = R_err_bound;
    end
    % Compute maximum translation distance
    [~,t_max,info] = compute_max_translation_distance(t_avg,A_c,B_c);
    if ~isempty(t_max) && (info.gap < 1e-6)
        t_err_max = getTranslationError(t_max,t_avg);
        fprintf("idx: %d, t_err_max: %3.2e.\n",id,t_err_max);
        correct_t_err_bound(id) = t_err_max;
    else
        t_err_bound = sqrt(info.f_sdp);
        fprintf("idx: %d, t_err_bound: %3.2e.\n",id,t_err_bound);
        correct_t_err_bound(id) = t_err_bound;
    end
end

save(fname_wrong,'abnormal','abnormal_obj_id','abnormal_smp_id','correct_R_err_bound','correct_t_err_bound');

