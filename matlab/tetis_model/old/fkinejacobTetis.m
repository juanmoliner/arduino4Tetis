function [JJ,J,pbe,Rbe,Tbe] = fkinejacobTetis(q)

% TETIS - Name of DORIS Manipulator

% JJ - Jacobian (Je)e (with respect to the end-effector) 6x5
% J - JJ Jacobian 5x5 (with the z-axis not controlled, as should be used for TETIS)
% pbe - position of end-effector with respect to the base
% Tbe - Forward kinematics
% q - joint space (joint 1 is the prismatic motion along the rail)

% parameters (FILL WITH THE CORRECT VALUES IN MM)

E2=52.55;
E3=320;
E4=225;
E5=167.25;
M3=55.775;
M4=55.775;
M5=57;

% calculations

q=q(:);
q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5);

Tb0 = [0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1];
T01 = [1 0 0 0;0 1 0 0;0 0 1 q1;0 0 0 1];
T12 = [-sin(q2) -cos(q2) 0 0; 0 0 -1 E2; cos(q2) -sin(q2) 0 0; 0 0 0 1];
T23 = [cos(q3) -sin(q3) 0 0;0 0 -1 0;sin(q3) cos(q3) 0 0;0 0 0 1];
T34 = [-cos(q4) sin(q4) 0 E3;-sin(q4) -cos(q4) 0 0;0 0 1 0;0 0 0 1];
T45 = [cos(q5) -sin(q5) 0 E4;0 0 -1 M5;sin(q5) cos(q5) 0 0;0 0 0 1];
T5e = [0 0 -1 -E5;0 1 0 0;1 0 0 0;0 0 0 1];

Tbe=Tb0*T01*T12*T23*T34*T45*T5e
Rbe=Tbe(1:3,1:3);
pbe=Tbe(1:3,4);

Rb0=Tb0(1:3,1:3);
Rb1=Rb0*T01(1:3,1:3);
Rb2=Rb1*T12(1:3,1:3);
Rb3=Rb2*T23(1:3,1:3);
Rb4=Rb3*T34(1:3,1:3);

z1_b=Rb0*T01(1:3,3);
z2_b=Rb1*T12(1:3,3);
z3_b=Rb2*T23(1:3,3);
z4_b=Rb3*T34(1:3,3);
z5_b=Rb4*T45(1:3,3);

pb0_b=Tb0(1:3,4);
p01_0=T01(1:3,4);
p12_1=T12(1:3,4);
p23_2=T23(1:3,4);
p34_3=T34(1:3,4);
p45_4=T45(1:3,4);

pb1_b=pb0_b+Rb0*p01_0;
pb2_b=pb1_b+Rb1*p12_1;
pb3_b=pb2_b+Rb2*p23_2;
pb4_b=pb3_b+Rb3*p34_3;
pb5_b=pb4_b+Rb4*p45_4;

p1e_b=pbe-pb1_b;
p2e_b=pbe-pb2_b;
p3e_b=pbe-pb3_b;
p4e_b=pbe-pb4_b;
p5e_b=pbe-pb5_b;

nulo=[0;0;0];
JJ=[z1_b skew(z2_b)*p2e_b skew(z3_b)*p3e_b skew(z4_b)*p4e_b skew(z5_b)*p5e_b;nulo z2_b z3_b z4_b z5_b];

JJ=[Rbe' zeros(3,3);zeros(3,3) Rbe']*JJ;
J=JJ(1:5,:);