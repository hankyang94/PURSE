clc
clear
close all

% fname = "pose_uncertain_cat_gnc_ball_5_0.10.mat";
fname = "pose_uncertain_ellinf_cat_gnc_ball_5_0.10.mat";
load(fname)

relaxpath = "../CertifiablyRobustPerception";
mosekpath   = '../../mosek';
addpath(genpath(relaxpath))
addpath(genpath(mosekpath))

n_samples = size(A,1);

for idx = 1:1
    T = squeeze(pose_avg(idx,:,:));
    R_avg = T(:,1:3);
    r_avg = R_avg(:);
    t_avg = T(:,end);
    A_c = squeeze(A(idx,:,:,:));
    B_c = squeeze(B(idx,:,:));
    % Compute maximum rotation distance
    [R_max,~,info] = compute_max_rotation_distance(R_avg,A_c,B_c);
    if ~isempty(R_max)
        R_err_max = getAngularError(R_max,R_avg);
        fprintf("idx: %d, R_err_max: %3.2e, R_err_max_smp: %3.2e.\n",idx,R_err_max,R_avg_smp_ucrt(idx));
    else
        R_err_bound = rad2deg(acos(1 - info.f_sdp/4));
        fprintf("idx: %d, R_err_bound: %3.2e, R_err_max_smp: %3.2e.\n",idx,R_err_bound,R_avg_smp_ucrt(idx));
    end
    
    % Compute maximum translation distance
    [~,t_max,info] = compute_max_translation_distance(t_avg,A_c,B_c);
    if ~isempty(t_max)
        t_err_max = getTranslationError(t_max,t_avg);
        fprintf("idx: %d, t_err_max: %3.2e, t_err_max_smp: %3.2e.\n",idx,t_err_max,t_avg_smp_ucrt(idx));
    else
        t_err_bound = sqrt(info.f_sdp);
        fprintf("idx: %d, t_err_bound: %3.2e, t_err_max_smp: %3.2e.\n",idx,t_err_bound,t_avg_smp_ucrt(idx));
    end
end


