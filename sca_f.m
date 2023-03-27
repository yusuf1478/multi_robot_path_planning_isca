function [bestSol, bestCost] = sca_f(maximumFitness, populationSize, dimension, maximumVelocity, ...
                            minimumVelocity, minimumTheta, maximumTheta, robotOld, dynamicOld, totalRobot, ...
                            robotRadius, robotGoal, totalStatic, staticRadius, static, totalDynamic, ...
                            dynamicRadius, costFunction)
    
    % control parameters
    fitcount = 0;
    myCost = costFunction;
    SearchAgents_no = populationSize;
    Max_iteration = maximumFitness;
    lb = minimumVelocity;
    ub = maximumVelocity;
    dim = dimension;
    a = 2;

    % initialize the set of random solutions
    X = rand(SearchAgents_no, dim) .* (ub - lb) + lb;
    Destination_position = zeros(1,dim);
    Destination_fitness = inf;
    Convergence_curve = zeros(1,Max_iteration);
    Objective_values = zeros(1,size(X,1));

    % calculate the fitness of the first set and find the best one
    for i = 1 : size(X,1)
        Objective_values(1,i) = myCost(X(i,:), minimumVelocity, maximumVelocity, minimumTheta, ...
                                    maximumTheta, robotOld, dynamicOld, totalRobot, robotRadius, robotGoal, ...
                                    totalStatic, staticRadius, static, totalDynamic, dynamicRadius);
        fitcount = fitcount + 1;
        if i == 1
            Destination_position = X(i,:);
            Destination_fitness = Objective_values(1,i);
        elseif Objective_values(1,i) < Destination_fitness
            Destination_position = X(i,:);
            Destination_fitness = Objective_values(1,i);
        end
        All_objective_values(1,i)=Objective_values(1,i);
    end

    % main loop
    t = 1;
    while fitcount <= Max_iteration

        % Eq. (3.4) in SCA paper,
        r1 = a - t * (a / Max_iteration); % r1 decreases linearly from a to 0

        % update the position of solutions with respect to destination
        for i = 1 : size(X,1) % in i-th solution
            for j = 1 : size(X,2) % in j-th dimension

                % update r2, r3, and r4 for Eq. (3.3) in SCA paper 
                r2 = (2*pi) * rand();
                r3 = 2*rand;
                r4 = rand();

                % Eq. (3.3) in SCA paper  
                if r4 < 0.5
                    % Eq. (3.1) in SCA paper 
                    X(i,j) = X(i,j) + (r1 * sin(r2) * abs(r3 * Destination_position(j) - X(i,j)));
                else
                    % Eq. (3.2) in SCA paper 
                    X(i,j) = X(i,j) + (r1 * cos(r2) * abs(r3 * Destination_position(j) - X(i,j)));
                end

            end
        end

        for i = 1 : size(X,1)

            % check if solutions go outside the search spaceand bring them back
            Flag4ub = X(i,:) > ub;
            Flag4lb = X(i,:) < lb;
            X(i,:) = (X(i,:) .* (~(Flag4ub+Flag4lb))) +ub .* Flag4ub + lb .* Flag4lb;

            % calculate the objective values
            Objective_values(1,i) = myCost(X(i,:), minimumVelocity, maximumVelocity, minimumTheta, ...
                                    maximumTheta, robotOld, dynamicOld, totalRobot, robotRadius, robotGoal, ...
                                    totalStatic, staticRadius, static, totalDynamic, dynamicRadius);
            fitcount = fitcount + 1;
            
            % update the destination if there is a better solution
            if Objective_values(1,i) < Destination_fitness
                Destination_position = X(i,:);
                Destination_fitness = Objective_values(1,i);
            end
        end
        
        % convergence
        Convergence_curve(t) = Destination_fitness;

        % Increase the iteration counter
        t = t + 1;
        
    end

    % bestsol and bestcost
    bestSol = Destination_position;
    bestCost = Destination_fitness;
    

end

