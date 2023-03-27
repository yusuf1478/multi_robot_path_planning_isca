function fitness = myCost(solution, minimumVelocity, maximumVelocity, minimumTheta, maximumTheta, robotOld, ...
                        dynamicOld, totalRobot, robotRadius, robotGoal, totalStatic, staticRadius, static, ...
                        totalDynamic, dynamicRadius)

for i = 1 : totalRobot
    

    % new positions of the robots relative to the corresponding solution
    % ===================================================================
    solution(2*i) = interp1( [minimumVelocity, maximumVelocity], [minimumTheta, maximumTheta], solution(2*i) );
    robotNew.x(i) = robotOld.x(i) + solution((2*i)-1) * cos(solution(2*i));
    robotNew.y(i) = robotOld.y(i) + solution((2*i)-1) * sin(solution(2*i));
    % ===================================================================


    % F1 (shortest distance)
    % ===================================================================
    fit(i) = sqrt( ( robotNew.x(i) - robotOld.x(i) )^2 + ( robotNew.y(i) - robotOld.y(i) )^2 ) + ...
             sqrt( ( robotNew.x(i) - robotGoal.x(i) )^2 + ( robotNew.y(i) - robotGoal.y(i) )^2 );
    % ===================================================================


    % F2 (avoiding static obstacles)
    % ===================================================================
    securityDistance = robotRadius(i);
    for j = 1 : totalStatic
        distanceToStatic = sqrt( ( robotNew.x(i) - static.x(j) )^2 + ( robotNew.y(i) - static.y(j) )^2 );
        if distanceToStatic < staticRadius(j) + robotRadius(i) + securityDistance
            fit(i) = fit(i) + 10000;
        end
    end
    % ===================================================================
    

    % F3 (avoiding dynamic obstacles)
    % ===================================================================
    for j = 1 : totalDynamic
        distanceToDynamic = sqrt( ( robotNew.x(i) - dynamicOld.x(j) )^2 + ( robotNew.y(i) - dynamicOld.y(j) )^2 );
        if distanceToDynamic < dynamicRadius(j) + robotRadius(i) + securityDistance
            fit(i) = fit(i) + 10000;
        end
    end
    % ===================================================================


    % F4 (avoiding other robots)
    % ===================================================================
    ol_x = robotOld.x;
    ol_y = robotOld.y;
    ol_x(i) = [];
    ol_y(i) = [];
    for j = 1 : (totalRobot-1)
        distanceToRobot = sqrt( ( robotNew.x(i) - ol_x(j) )^2 + ( robotNew.y(i) - ol_y(j) )^2 );
        if distanceToRobot < robotRadius(i)*3
            fit(i) = fit(i) + 10000;
        end
    end
    % ===================================================================


end
    

% Fit = F1 + F2 + F3 + F4
% ===================================================================
fitness = sum(fit);
% ===================================================================

end




