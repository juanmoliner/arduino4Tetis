%
% Tetis arm example
%

close all;

% parameters
E2=52.55;
E3=320;
E4=225;
E5=167.25;
M3=55.775;
M4=55.775;
M5=57;


%
y=[0;1;0];
z=[0;0;1];
h1=z;  h2=-y;  h3=h2;
h4=z;  
% end-efector
h5=z;


H=[h1 h2 h3 h4 h5];
p01=zeros(3,1); p12=p01;   p23=[E3;0;0];
p34=[E4;0;0];
p45=[E5;0;M5];

P=[p01 p12 p23 p34 p45];
type=[0 0 0 0 0]; % 6R robot
n=5;

scale=0.3;plotsize=500;
fignum=1;figure(fignum)
theta=1;
while size(theta)>0
  theta=input(['enter theta vector (',num2str(n),'x1): ']);
  if size(theta)>0
    clf(fignum);
    [R,p]=fwdkin(theta,type,H,P,n);
    disp(R);disp(p)
    showarm(theta,type,H,P,n,fignum,scale,plotsize);
    disp('press any key to continue ...');
    pause;
  end
end
