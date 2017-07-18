function showatt(R,p,fignum,scale)
  
  figure(fignum)
  hold on
  plot3(p(1),p(2),p(3),'*');
%  plot3(0,0,0,'o');
  plot3([p(1) p(1)+R(1,1)*scale],[p(2) p(2)+R(2,1)*scale], ...
      [p(3) p(3)+R(3,1)*scale],'r');
  plot3([p(1) p(1)+R(1,2)*scale],[p(2) p(2)+R(2,2)*scale], ...
      [p(3) p(3)+R(3,2)*scale],'b');
  plot3([p(1) p(1)+R(1,3)*scale],[p(2) p(2)+R(2,3)*scale], ...
      [p(3) p(3)+R(3,3)*scale],'k');
