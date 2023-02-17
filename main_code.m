%
% Copyright (c) 2023, all rights reserved.
%
% Project Title: Multi-strategy and self-adaptive differential sine-cosine
%                algorithm for multi-robot path planning
% Developers: Rustu Akay, Mustafa Yusuf Yildirim
% 
% Contact Info: myyildirim@erciyes.edu.tr
%

clc;
clear;
close all;


% data of scenario 1
% ===================================================================
totalRobot = 6; % number of robot
robotRadius = ones(1, totalRobot); % radius of robot
robotStart.x = [5 25 45 95 85 95]; % start positions (x) of robots
robotStart.y = [95 10 75 95 40 5]; % start positions (y) of robots
robotGoal.x =  [40 25 10 65 40 65]; % goal positions (x) of robots
robotGoal.y =  [13 97 55 15 40 75]; % goal positions (y) of robots
totalStatic = 7; % number of static obstacles
staticRadius = [4 3 3 5 4 5 3]; % radius of static obstacles
static.x = [10 25 25 40 35 75 85]; % positions (x) of static obstacles
static.y = [80 90 30 65 25 40 15]; % positions (y) of static obstacles
totalDynamic = 3;  % number of dynamic obstacles
dynamicRadius = 1.5 * ones(1, totalDynamic); % radius of dynamic obstacles
dynamicStart.x = [15 55 75]; % start positions (x) of dynamic obstacles
dynamicStart.y = [40 20 90]; % start positions (y) of dynamic obstacles
dynamicGoal.x = [35 50 95]; % goal positions (x) of dynamic obstacles
dynamicGoal.y = [85 50 75]; % goal positions (y) of dynamic obstacles
dynamicVelocity = [0.5 0.45 1.2]; % velocities of dynamic obstacles
axisParameter = 100; % visualization scale (100 x 100)
% ===================================================================


% data of scenario 2 
% ===================================================================
% totalRobot = 12; % number of robot
% robotRadius = ones(1, totalRobot); % radius of robot
% robotStart.x = [10 20 50 60 70 90 110 110 110 120 180 180]; % start positions (x) of robots
% robotStart.y = [60 190 150 60  170 120 60 160 180 120 80 160]; % start positions (y) of robots
% robotGoal.x =  [30 20 50 30 70 140 190 150 160 60 180 180]; % goal positions (x) of robots
% robotGoal.y =  [90 40 30 60 90 50  110 180 140 10 150 180]; % goal positions (y) of robots
% totalStatic = 14; % number of static obstacles
% staticRadius = [4 4 3 4 3 3 3 3 5 5 3 3 3 3]; % radius of static obstacles
% static.x = [20 20 40 50 50 70 70 70 110 140 140 160 180 180]; % positions (x) of static obstacles
% static.y = [80 160 60 60 90 20 110 150 100 160 175 90 100 170]; % positions (y) of static obstacles
% totalDynamic = 6; % number of dynamic obstacles
% dynamicRadius = 1.5 * ones(1, totalDynamic); % radius of dynamic obstacles
% dynamicStart.x = [10  40 60  80 120 170]; % start positions (x) of dynamic obstacles
% dynamicStart.y = [150 90 160 85 50  120]; % start positions (y) of dynamic obstacles
% dynamicGoal.x = [30  60  80  100 140 190]; % goal positions (x) of dynamic obstacles
% dynamicGoal.y = [100 130 120 30  100 125]; % goal positions (y) of dynamic obstacles
% dynamicVelocity = [0.5 0.5 0.6 0.3 0.4 0.25]; % velocities of dynamic obstacles
% axisParameter = 200; % visualization scale (200 x 200)
% ===================================================================


% initial visualization
% ===================================================================
alpha = linspace(0, 2*pi, 100);
set(gcf, 'WindowState', 'maximized');
plt1 = plot(-7,-7,'o','MarkerFaceColor',[0.5 0.5 0.5],'MarkerEdgeColor','k','MarkerSize',12);
hold on;
plt2 = plot(-8,-8,'o','MarkerFaceColor',[0.5 0.7 0.8],'MarkerEdgeColor','k','MarkerSize',12);
plt3 = plot(-9,-9,'o','MarkerFaceColor',[0.1 0.2 0.9],'MarkerEdgeColor','k','MarkerSize',12);
for i = 1 : totalStatic
    fill(static.x(i) + staticRadius(i) * cos(alpha), static.y(i) + staticRadius(i) * sin(alpha), [0.5 0.5 0.5]);
end
for i = 1 : totalRobot
    plt11(i) = fill(robotStart.x(i) + robotRadius(i) * cos(alpha), robotStart.y(i) + robotRadius(i) * sin(alpha), [0.5 0.7 0.8]);
    plt4 = plot(robotGoal.x(i), robotGoal.y(i),'x','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',10,'LineWidth',2);
    plt5 = line([robotStart.x(i) robotGoal.x(i)], [robotStart.y(i) robotGoal.y(i)],'Color','black','LineStyle','--');
end
for i = 1 : totalDynamic
    plt12(i) = fill(dynamicStart.x(i) + dynamicRadius(i) * cos(alpha), dynamicStart.y(i) + dynamicRadius(i) * sin(alpha), [0.1 0.2 0.9]);
    plt6 = plot(dynamicGoal.x(i), dynamicGoal.y(i),'x','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',10,'LineWidth',2);
    plt7 = line([dynamicStart.x(i) dynamicGoal.x(i)], [dynamicStart.y(i) dynamicGoal.y(i)],'Color','blue','LineStyle','--');
end
axis equal;
axis([-5 (axisParameter+5) 0 (axisParameter)]);
pause(1);
% ===================================================================


% ideal path distance of robots
% ===================================================================
for i = 1 : totalRobot
    idealDistance(i) = sqrt( ( robotStart.x(i) - robotGoal.x(i) )^2 + ( robotStart.y(i) - robotGoal.y(i) )^2 );
end
% ===================================================================


% optimization control parameters
% ===================================================================
costFunction = @myCost;
maximumFitness = 1000; 
populationSize = 30;
dimension = 2 * totalRobot;
minimumVelocity = 1;
maximumVelocity = 1.5;
minimumTheta = 0;
maximumTheta = 2*pi;
% ===================================================================


% path planning initial setting
% ===================================================================
step = 1;
robotOld.x = robotStart.x;
robotOld.y = robotStart.y;
robotPath.x = [robotOld.x];
robotPath.y = [robotOld.y];
dynamicOld.x = dynamicStart.x;
dynamicOld.y = dynamicStart.y;
dynamicPath.x = [dynamicOld.x];
dynamicPath.y = [dynamicOld.y];
% ===================================================================


% distances to goals of robots
% ===================================================================
for i = 1 : totalRobot
    distanceToGoal(i) = sqrt( ( robotOld.x(i) - robotGoal.x(i) )^2 + ( robotOld.y(i) - robotGoal.y(i) )^2 );
end
stoppingCriteria = max(distanceToGoal);
% ===================================================================


% main loop
% ===================================================================
tic;
while stoppingCriteria > 1
    
    % The robot and dynamic obstacle in the relevant step are deleted so that they are not drawn on top of each other
    % ===================================================================
    delete(plt11);
    delete(plt12);
    % ===================================================================


    % optimization loop (SCA)
    % ===================================================================
%     [bestSol, bestCost] = sca_f(maximumFitness, ...
%                                 populationSize, ...
%                                 dimension, ...
%                                 maximumVelocity, ...
%                                 minimumVelocity, ...
%                                 minimumTheta, ...
%                                 maximumTheta, ...
%                                 robotOld, ...
%                                 dynamicOld, ...
%                                 totalRobot, ...
%                                 robotRadius, ...
%                                 robotGoal, ...
%                                 totalStatic, ...
%                                 staticRadius, ...
%                                 static, ...
%                                 totalDynamic, ...
%                                 dynamicRadius, ...
%                                 costFunction);
    % ===================================================================


    % optimization loop (sdSCA)
    % ===================================================================
    [bestSol, bestCost] = sdsca_f(maximumFitness, ...
                                populationSize, ...
                                dimension, ...
                                maximumVelocity, ...
                                minimumVelocity, ...
                                minimumTheta, ...
                                maximumTheta, ...
                                robotOld, ...
                                dynamicOld, ...
                                totalRobot, ...
                                robotRadius, ...
                                robotGoal, ...
                                totalStatic, ...
                                staticRadius, ...
                                static, ...
                                totalDynamic, ...
                                dynamicRadius, ...
                                costFunction);
    % ===================================================================


    % total cost
    % ===================================================================
    cost(step) = bestCost;
    % ===================================================================


    % motions of robots
    % ===================================================================
    for i = 1 : totalRobot
        bestSol(2*i) = interp1( [minimumVelocity, maximumVelocity], [minimumTheta, maximumTheta], bestSol(2*i) );
        distanceToGoal(i) = sqrt( ( robotOld.x(i) - robotGoal.x(i) )^2 + ( robotOld.y(i) - robotGoal.y(i) )^2 );
        if distanceToGoal(i) > 1
            robotNew.x(i) = robotOld.x(i) + bestSol((2*i)-1) * cos(bestSol(2*i));
            robotNew.y(i) = robotOld.y(i) + bestSol((2*i)-1) * sin(bestSol(2*i));
        end
        % visualization
        plt11(i) = fill( robotNew.x(i) + robotRadius(i) * cos(alpha), robotNew.y(i) + robotRadius(i) * sin(alpha), [0.5 0.7 0.8] );
        line([robotOld.x(i) robotNew.x(i)],[robotOld.y(i) robotNew.y(i)],'Color','red','LineStyle','--');
    end
    % ===================================================================


    % motion of dynamic obstacles
    % ===================================================================
    for i = 1 : totalDynamic
        dynamicDistanceToGoal(i) = sqrt( ( dynamicOld.x(i) - dynamicGoal.x(i) )^2 + ( dynamicOld.y(i) - dynamicGoal.y(i) )^2 );
        if dynamicDistanceToGoal(i) > 1
            slope(i) = atan2( dynamicGoal.y(i) - dynamicOld.y(i) , dynamicGoal.x(i) - dynamicOld.x(i) );
            dynamicNew.x(i) = dynamicOld.x(i) + dynamicVelocity(i) * cos(slope(i));
            dynamicNew.y(i) = dynamicOld.y(i) + dynamicVelocity(i) * sin(slope(i));
        end
        % visualization
        plt12(i) = fill( dynamicNew.x(i) + dynamicRadius(i) * cos(alpha), dynamicNew.y(i) + dynamicRadius(i) * sin(alpha), [0.1 0.2 0.9] );
    end
    pause(0.01);
    % ===================================================================


    % Updating positions and paths of robots and dynamic obstacles 
    % ===================================================================
    robotOld.x = robotNew.x;
    robotOld.y = robotNew.y;
    robotPath.x = [robotPath.x; robotNew.x];
    robotPath.y = [robotPath.y; robotNew.y];
    dynamicOld.x = dynamicNew.x;
    dynamicOld.y = dynamicNew.y;
    dynamicPath.x = [dynamicPath.x; dynamicNew.x];
    dynamicPath.y = [dynamicPath.y; dynamicNew.y];
    % ===================================================================
    

    % distances to goals of robots
    % ===================================================================
    for i = 1 : totalRobot
        distanceToGoal(i) = sqrt( ( robotOld.x(i) - robotGoal.x(i) )^2 + ( robotOld.y(i) - robotGoal.y(i) )^2 );
    end
    stoppingCriteria = max(distanceToGoal);
    % ===================================================================


    % step count is increased by one
    % ===================================================================
    step = step + 1;
    % ===================================================================

end
% ===================================================================


% Execution times and paths
% ===================================================================
executionTime = toc;
paths_x = robotPath.x;
paths_y = robotPath.y;
% ===================================================================

        
        