clc
clear
close all

fname = "../6D_Pose/duck_sample_radius-maxp.mat";
load(fname)

relaxpath = "../CertifiablyRobustPerception";
mosekpath   = '../mosek';
addpath(genpath(relaxpath))
addpath(genpath(mosekpath))

t_avg = t_avg(:);
% Compute maximum rotation distance
[R_max,t_R_max,info] = compute_max_rotation_distance(R_avg,A,B);
if ~isempty(R_max)
    R_err_max = getAngularError(R_max,R_avg);
    fprintf("R_err_max: %3.2e.\n",R_err_max);
else
    R_err_bound = rad2deg(acos(1 - info.f_sdp/4));
    fprintf("R_err_bound: %3.2e.\n",R_err_bound);
end

% Compute maximum translation distance
[R_t_max,t_max,info] = compute_max_translation_distance(t_avg,A,B);
if ~isempty(t_max)
    t_err_max = getTranslationError(t_max,t_avg);
    fprintf("t_err_max: %3.2e.\n",t_err_max);
else
    t_err_bound = sqrt(info.f_sdp);
    fprintf("t_err_bound: %3.2e.\n",t_err_bound);
end

%% plot
duck_cloud = pcread('../6D_Pose/data/bop/lmo/models/obj_000009.ply');

clc
close all
figure;
scale = 0.2;
mArrow3(zeros(3,1),scale*[1;0;0],color='red');
hold on
mArrow3(zeros(3,1),scale*[0;1;0],color='green');
hold on
mArrow3(zeros(3,1),scale*[0;0;1],color='blue');
hold on
duck_avg_loc = (R_avg * (double(duck_cloud.Location)/1000)' + t_avg)';
pcshow(duck_avg_loc,duck_cloud.Color,'BackgroundColor','white')
hold on
duck_Rmax_loc = (R_max * (double(duck_cloud.Location)/1000)' + t_R_max)';
pcd1 = pcshow(duck_Rmax_loc,'blue','BackgroundColor','white');
alpha(pcd1,0.2)
hold on
duck_tmax_loc = (R_t_max * (double(duck_cloud.Location)/1000)' + t_max)';
pcd2 = pcshow(duck_tmax_loc,'red','BackgroundColor','white');
alpha(pcd2,0.2)

save('duck_sample_worst.mat','R_max','t_R_max','R_t_max','t_max')



