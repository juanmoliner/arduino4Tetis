function showarm(theta,type,H,P,n,fignum,scale,plotsize)

%  scale=0.1;
%  scale=0.3;
  figure(fignum);
% inertial frame
  hold on
  p_i_1=zeros(3,1);
  showatt(eye(3,3),p_i_1,fignum,scale);

  if type(1) == 0
    R=expm(crossmat(H(1:3,1))*theta(1));
    p_i=P(1:3,1);
  else
    R=eye(3,3);
    p_i=P(1:3,1)+theta(1)*H(1:3,1);
  end

  for i=2:n
    ll=line([p_i_1(1) p_i(1)],...
	[p_i_1(2) p_i(2)],[p_i_1(3) p_i(3)]);
    set(ll,'LineWidth',2); set(ll,'Color',[.5 .6 .5]);
    showatt(R,p_i,fignum,scale);
    p_i_1=p_i;
    if type(i) ==0
      p_i=p_i+R*P(1:3,i);
      R=R*expm(crossmat(H(1:3,i))*theta(i));
    else
      p_i=p_i+R*(P(1:3,i)+theta(i)*H(1:3,i));
      R=R;
    end
%    disp(p_i_1); disp(p_i)
  end
  ll=line([p_i_1(1) p_i(1)],...
      [p_i_1(2) p_i(2)],[p_i_1(3) p_i(3)]);
  set(ll,'LineWidth',2); set(ll,'Color',[.5 .6 .5]);
  showatt(R,p_i,fignum,scale);

%  plotsize=3;
%  plotsize=1;
  axis([-plotsize plotsize -plotsize plotsize -plotsize plotsize]);
  axis('square');
  view(60,50);
  title('red = x, blue = y, black = z')
  grid;
  hold off