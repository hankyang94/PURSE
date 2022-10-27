clc
clear
close all

fname = "pose_samples_10000000.mat";
load(fname)

addpath(genpath("MyCrustOpen"))
addpath(genpath("MyCrust"))

angles = samples(:,1:3);
locs = samples(:,4:6);

% out = MyCrustOpen(angles);
% [out,~] = MyRobustCrust(angles);
% out = delaunay(angles);
k = boundary(angles,0.2);

figure;
% trisurf(k,angles(:,1),angles(:,2),angles(:,3),'facecolor','red','edgecolor','c')
trisurf(k,angles(:,1),angles(:,2),angles(:,3))
xlabel('roll')
ylabel('pitch')
zlabel('yaw')
axis vis3d


k = boundary(locs,0.2);

figure;
trisurf(k,locs(:,1),locs(:,2),locs(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
axis vis3d