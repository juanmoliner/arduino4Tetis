delete(instrfindall);
clc;
clear qd1 qd2 qd3 qd4 q1 q2 q3 q4 u1 u2 u3 u4 a temp_axisQ countQ tplotQ temp_axisX countX tplotX temp_axisU countU tplotU tinitsimu tsimu

%a = serial('/dev/tty.usbmodemFD121','BaudRate',57600);
% a = serial('/dev/tty.usbmodemFA131','BaudRate',115200);
% a = serial('/dev/tty.usbmodemFD121','BaudRate',921600);
 a = serial('/dev/tty.usbmodemFA131','BaudRate',921600);
fopen(a);

qd1 = [];
qd2 = [];
qd3 = [];
qd4 = [];

q1 = [];
q2 = [];
q3 = [];
q4 = [];
temp_axisQ = [];
countQ = 0;
tplotQ = 0;


xd1 = [];
xd2 = [];
xd3 = [];
xd4 = [];

x1 = [];
x2 = [];
x3 = [];
x4 = [];
temp_axisX = [];
countX = 0;
tplotX = 0;


u1 = [];
u2 = [];
u3 = [];
u4 = [];
temp_axisU = [];
countU = 0;
tplotU = 0;

tinitsimu = now;
tsimu = 0;


while ~ strcmp(line,'----------------- RESTART ------------------')
    line = fgetl(a);
    line = line(1:end-1);
    disp('flushing data from last execution: ');
    disp(line);
end


while tsimu < 60
    tsimu = (now - tinitsimu)* 24 * 60 * 60;
    if a.BytesAvailable > 0
        line = fgetl(a);
        data = strsplit(line,' ');
        disp(line);
        if strcmp(data(1),'ToMatlabQ:')
            if countQ == 0
              tinitplotQ = now;
            end
%             tplotQ = (now - tinitplotQ)* 24 * 60 * 60;
            countQ = countQ + 1;
            
            qd1 = [qd1 str2double(data(2))];
            qd2 = [qd2 str2double(data(3))];
            qd3 = [qd3 str2double(data(4))];
            qd4 = [qd4 str2double(data(5))];
            
            q1 = [q1 str2double(data(6))];
            q2 = [q2 str2double(data(7))];
            q3 = [q3 str2double(data(8))];
            q4 = [q4 str2double(data(9))];
            
            tq = str2double(data(10));
 
            temp_axisQ = [temp_axisQ str2double(data(10)) / 1000.0];

        elseif strcmp(data(1),'ToMatlabX:');
            if countX == 0
                tinitplotX = now;
            end
%             tplotX = (now - tinitplotX)* 24 * 60 * 60;
            countX = countX + 1;
            
            xd1 = [xd1 str2double(data(2))];
            xd2 = [xd2 str2double(data(3))];
            xd3 = [xd3 str2double(data(4))];
            xd4 = [xd4 str2double(data(5))*180/pi];
            
            x1 = [x1 str2double(data(6))];
            x2 = [x2 str2double(data(7))];
            x3 = [x3 str2double(data(8))];
            x4 = [x4 str2double(data(9))*180/pi];
 
            temp_axisX = [temp_axisX str2double(data(10)) / 1000.0];
 
        elseif strcmp(data(1),'ToMatlabU:');
            if countU == 0
              tinitplotU = now;
            end
%             tplotU = (now - tinitplotU)* 24 * 60 * 60;
            countU = countU + 1;
            
            u1 = [u1 str2double(data(2))];
            u2 = [u2 str2double(data(3))];
            u3 = [u3 str2double(data(4))];
            u4 = [u4 str2double(data(5))];

            temp_axisU = [temp_axisU str2double(data(6)) / 1000.0];
        end
    end
end

if countQ > 0
    figQ = figure('name','Position in space of the joints');
    set(gcf,'position',[20 20 600 350])
    hold on;
    figure(figQ);
    Q(1) = subplot(2,2,1);
    plot(temp_axisQ,qd1,'g',temp_axisQ,q1,'r');
    Q(2) = subplot(2,2,2);
    plot(temp_axisQ,qd2,'g',temp_axisQ,q2,'r');
    Q(3) = subplot(2,2,3);
    plot(temp_axisQ,qd3,'g',temp_axisQ,q3,'r');
    Q(4) = subplot(2,2,4);
    plot(temp_axisQ,qd4,'g',temp_axisQ,q4,'r');
    title(Q(1),'Joint 1');
    title(Q(2),'Joint 2');
    title(Q(3),'Joint 3');
    title(Q(4),'Joint 4');
    for i=1:4
        ylabel(Q(i),{'Angle [deg]'})
        xlabel(Q(i),{'Time [s]'})
        legend(Q(i),'qd(t)','q(t)')
        grid(Q(i),'on')
    end
end


if countU > 0
    figU = figure('name','Control signal');
    set(gcf,'position',[20 620 1300 200])
    hold on;
    figure(figU);
    U(1) = subplot(2,2,1);
    plot(temp_axisU,u1,'b');
    U(2) = subplot(2,2,2);
    plot(temp_axisU,u2,'b');
    U(3) = subplot(2,2,3);
    plot(temp_axisU,u3,'b');
    U(4) = subplot(2,2,4);
    plot(temp_axisU,u4,'b');
    title(U(1),'Joint 1');
    title(U(2),'Joint 2');
    title(U(3),'Joint 3');
    title(U(4),'Joint 4');
    for i=1:4
        ylabel(U(i),{'Velocity   [deg/s]'})
        xlabel(U(i),{'Time [s]'})
        legend(U(i),'u(t)')
        grid(U(i),'on')
    end
end

if countX > 0
    figX = figure('name','Position in space of the actuator');
    set(gcf,'position',[680 20 600 350])
    hold on;
    figure(figX);
    X(1) = subplot(2,2,1);
    plot(temp_axisX,xd1,'g',temp_axisX,x1,'r');
    X(2) = subplot(2,2,2);
    plot(temp_axisX,xd2,'g',temp_axisX,x2,'r');
    X(3) = subplot(2,2,3);
    plot(temp_axisX,xd3,'g',temp_axisX,x3,'r');
    X(4) = subplot(2,2,4);
    plot(temp_axisX,xd4,'g',temp_axisX,x4,'r');
    title(X(1),'X');
    title(X(2),'Y');
    title(X(3),'Z');
    title(X(4),'Pitch');
    for i=1:3
        ylabel(X(i),{'Position[mm]'})
        xlabel(X(i),{'Time [s]'})
        legend(X(i),'xd(t)','x(t)')
        grid(X(i),'on')
    end
    ylabel(X(4),{'Angle[deg]'})
    xlabel(X(4),{'Time [s]'})
    legend(X(4),'xd(t)','x(t)')
    grid(X(i),'on')
    
    figXD = figure('name','Trajectory in x-z plane');
    hold on;
    grid on;
    plot(x1,x3,xd1,xd3);
    ylabel('X [mm]')
    xlabel('Z [mm]')
    legend('x(t)','xd(t)')
    
end
% 
% fclose(a);
% delete(a);
% clear a;