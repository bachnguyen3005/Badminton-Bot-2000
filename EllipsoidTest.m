%% Generate Robot Model

clc
%BadmintonBot = RacketBot();

%% Plot Racket Bot
% q = zeros(1, 7);
% hold on;
% axis([-4 4 -4 4 0 4]);
% view(3);
% BadmintonBot.model.animate(q);

%% Place Cube in Environment
% Cube Details
xlength = 0.5;
ylength = 0.5;
zlength = 1;
center = [0.6 0.7 1];

% One side of the cube
[Y,Z] = meshgrid(-ylength:0.05:ylength,-zlength:0.05:zlength);
sizeMat = size(Y);
X = repmat(xlength,sizeMat(1),sizeMat(2));

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2)]; 
             % ; cubePoints * roty(pi/2) ...
             % ; cubePoints * roty(-pi/2)];         

% Plot the cube's point cloud         
cubePoints = cubePoints + repmat(center,size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');

%% Move Racket Bot while checking for Collisions

%qlim = r.model.qlim;

for q = 0:-0.019:-2
    hold on;
    r.model.animate([q zeros(1,6)]);
    qnow = r.model.getpos();
    [check,logMessage] = ellipsoidCollisionChecking(r, cubePoints); % IMPORTANT LINE %
    disp(logMessage);
    if check == 1
        disp(logMessage);
        break
    end
end