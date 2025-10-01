function [xyz,ypr,R]=ew6toBlender(ew6Save,cam)

% function [xyz,ypr,R]=ew6toBlender(ew6Save,cam)
%
% Get the camera position and orientation from an ew6 saved data file for
% use in Blender
% 
% Inputs: ew6Save = an easyWand6 saved data file
%         cam = the number of the camera to get the information for
%
% xyz = camera position
% ypr = camera rotation about [x,y,z] in the standard Blender Euler order
% R = the scaling ratio between image plane distance and size
%
% Steps for setting up your Blender camera
% 0) Run ew6toBlender and collect the 3 output variables
% 1) Create a new camera in Blender, set its position and rotation to [0,0,0] and [0,0,0]
% 2) Create a new image mesh plane (Add --> Image --> Mesh plane) and attach your camera image or video to it
% 3) Set the mesh plane position to [0,0,-10] and its rotation to [0,0,0]
% 4) Make the mesh plane a child object of the camera
% 5) Set the camera's Location xyz to the xyz values from ew6toBlender
% 6) Set the camera's Rotation xyz to the ypr values from ew6toBlender
% 7) Set the mesh plane scale and position from R, such that the mesh plane scale is the absolute mesh plane Z position divided by R
% 8) If your camera has a principal point offset away from center you can translate the mesh plane in X and Y scaled by R to match things up
%
% Blender does not seem to be able to apply pinhole model lens undistortion
% to cameras so you need to create an undistorted image or video to attach
% to the mesh plane if your camera has noticeable distortion
%
%
% Ty Hedrick 2025-10-01

i=cam; % the camera from ew6Save that will be processed

% xyz
xyz=ew6Save.data.extrinsics.positions{i};
foo=[1,0,0;0,1,0;0,0,-1];
foo2=[1,0,0;0,-1,0;0,0,-1];
pose=-ew6Save.data.extrinsics.poses{i}'*foo;
xyz=(-ew6Save.data.extrinsics.poses{i}'*xyz')'; % transform position for pose

% ypr
R2=[1,0,0;0,-1,0;0,0,-1];
R3=[-1,0,0;0,-1,0;0,0,1];
T=ew6Save.data.extrinsics.poses{i}'*foo2;
alpha=atan2(T(2,1),T(1,1));
beta=atan2(-T(3,1), (T(3,2)^2+T(3,3)^2)^0.5);
gamma=atan2(T(3,2),T(3,3));
ypr=rad2deg([gamma,beta,alpha]);

% scale
F=ew6Save.data.intrinsics(i).FocalLength(1);
R=F/ew6Save.data.intrinsics(i).imageSize(2);