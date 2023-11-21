function [qMatrix] = MiddleCourtLow(r,xPos,angle,velocity)
%
% q1C= zeros(1,7);
% q2C = [-2 deg2rad([0 0 0 0 0 -30])];
qHit = [-0.75 deg2rad([0 0 0 -90 0 90])];
%
%[qMatrix qWayPoints] = CollisionAvoidance(r,centerpnt,side,q1C,q2C);
%
% for i = 1:size(qMatrix,1)
% r.model.animate(qMatrix(i,:));   
% pause(0.03)
% end

g = 9.81;
theta = deg2rad(angle);
tEnd = 2*velocity*sin(theta)/g;
%t_end = t1_end + t2_end;
t = linspace(0,tEnd);

y = velocity*t*cos(theta)-1;
z = 0.1+velocity*t*sin(theta)-0.5*g*t.^2;
x = xPos + 0.*t;
% save("model3.mat","y","z");
%plot3(x,y,z,'o');

% Load the pre-defined path
% load('model3.mat');
% x = zeros(1,100);

% Find the contact point => num
num = 65;
% Move the end-effector to that point

T1 = [eye(3) [x(1,num) y(1,num) z(1,num)-0.1]'; zeros(1,3) 1]; %Pose when contact the shuttle cock

T2 = [eye(3) [x(1,num) y(1,num+6) z(1,num+6)-0.1]'; zeros(1,3) 1]; %Swing pose  

M = [1 1 1 1 1 1]; %masking matrix

q1 = r.model.ikcon(T1,qHit);                    % Solve for joint angles
q2 = r.model.ikcon(T2,qHit);

% Move to swing pose
qNow = r.model.getpos();
qTraj = jtraj(qNow,q1,50);
for i = 1:50
r.model.animate(qTraj(i,:));
%T0 = r.model.fkine(qMatrix0(i,:)).T;
%line1 = plot3(T0(1,4), T0(2,4), T0(3,4), '*');
pause(0.01);
end
% Swing pose by RMRC-Resolved motion rate control => qMatrix 

steps = num;

x1 = [x(1,num) y(1,num) (z(1,num)-0.1) 0 0 0]';
x2 = [x(1,num) y(1,num+6) (z(1,num+6)-0.1) 0 0 0]';
deltaT = 0.05;                                        % Discrete time step

xR = zeros(6,steps);

s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    xR(:,i) = x1*(1-s(i)) + s(i)*x2; 
 
end

qMatrix = [nan(steps,6) zeros(steps,1)];

qMatrix(1,:) = q1;
%r.model.ikcon(T1);                 

for i = 1:steps-1
    xdot = (xR(:,i+1) - xR(:,i))/deltaT;                             
    J = r.model.jacob0(qMatrix(i,:));            
    J = J(1:6,1:6);                           
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix(i+1,1:4) =  qMatrix(i,1:4) + deltaT*qdot(1:4,1)';
    
    qMatrix(i+1,5) = deg2rad(-80)-deg2rad(90)/65*i;
    qMatrix(i+1,6) = 0;
    qMatrix(i+1,7) = pi/2;

end

% Shuttlecock start flying with pre-defined path to the contact point
bricknum = [xPos,-1.5,0.1];
[f,v,data] = plyread('shuttleCock.PLY','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
BrickVertexCount = size(v,1);
BrickMesh_h = trisurf(f,v(:,1)+bricknum(1,1),v(:,2)+bricknum(1,2), v(:,3)+bricknum(1,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

counter = num;
for i=1:num    
    brick = transl(x(1,i),y(1,i),z(1,i))*trotx(i*pi/100);

    BrickPose = brick ;

    %MOVE THE BRICK. Update the point then multiply it to the vertices
    UpdatedPoints = [BrickPose * [v,ones(BrickVertexCount,1)]']';

    %The vertices are all the rows and 1 to 3 columns of the UpdatedPoints
    BrickMesh_h.Vertices = UpdatedPoints(:,1:3);

    
        r.model.animate(qMatrix(counter,:));
        T = r.model.fkine(qMatrix(counter,:)).T;
        line1 = plot3(T(1,4), T(2,4), T(3,4)+0.1,'o','MarkerSize',1,'Color','m');
       
        counter = counter - 1;

    h = plot3(x(1,i),y(1,i),z(1,i),'o','MarkerSize',3,'Color','r');

    pause(0.01);
   
end

try delete(h); 
catch Me
end
% Shuttlecock by backward
counter = 1;
for i=num:-1:1    
    brick = transl(x(1,i),y(1,i),z(1,i))*trotx(i*pi/100);

    BrickPose = brick ;

    %MOVE THE BRICK. Update the point then multiply it to the vertices
    UpdatedPoints = [BrickPose * [v,ones(BrickVertexCount,1)]']';

    %The vertices are all the rows and 1 to 3 columns of the UpdatedPoints
    BrickMesh_h.Vertices = UpdatedPoints(:,1:3);

    
        r.model.animate(qMatrix(counter,:));
        T = r.model.fkine(qMatrix(counter,:)).T;
        %line1 = plot3(T(1,4), T(2,4), T(3,4), '*');
       
        counter = counter + 1;
 
    pause(0.005);
end

% Move back to the initial pose
qCurrent =r.model.getpos();
qTraj = jtraj(qCurrent,zeros(1,7),50);
for i = 1:50 
r.model.animate(qTraj(i,:));
pause(0.01);
end


end