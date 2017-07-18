%MDL_tetis2 Create model of Tetis planar manipulator
%
%      mdl_tetis2
%
% Script creates the workspace variable dia10 which describes the 
% kinematic and dynamic characteristics of a Motoman DIA10  manipulator
% using standard DH conventions.
% The model includes armature inertia and gear ratios.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%   qn         arm is at a nominal non-singular configuration
%

E2=52.55;
E3=320;
E4=225;
E5=167.25;
M3=55.775;
M4=55.775;
M5=57;

clear L
%             th    d       a         alpha
L(1) = Link([ 0 	0       0	      pi/2    0], 'standard');
L(2) = Link([ 0     0	    E3        0       0], 'standard');
L(3) = Link([ 0     0	    E4        0       0], 'standard');
L(4) = Link([ 0     -M5     E5       -pi/2    0], 'standard');


Tetis= SerialLink(L, 'name', 'Tetis - Planar', ...
    'manufacturer', 'GSCAR', 'comment', '');


%
% some useful poses
%
qz = [0 0 0 0 ]; % zero angles, L shaped pose
qr = [0 -pi/4 pi/2 -pi/4 ]; % ready pose, arm up
qc = [0 -pi/2 0 0];
qh = [pi 0 pi pi];

clear L
