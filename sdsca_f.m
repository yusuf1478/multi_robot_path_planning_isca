function [bestSol, bestCost] = sdsca_f(maximumFitness, populationSize, dimension, maximumVelocity, ...
                            minimumVelocity, minimumTheta, maximumTheta, robotOld, dynamicOld, totalRobot, ...
                            robotRadius, robotGoal, totalStatic, staticRadius, static, totalDynamic, ...
                            dynamicRadius, costFunction)
    
        % control parameters
        fitcount = 0;
        myCost = costFunction;
        npop = populationSize;
        maxit = maximumFitness;
        lb = minimumVelocity;
        ub = maximumVelocity;
        dim = dimension;
        a = 2;
        pa_min = 0.1;
        FF = 0.8;
        CR = 0.95;
    
        % strategy pool initial setting
        counter = zeros(1, npop);
        K = 4; % number of strategy
        delta_fes = zeros(1,K);
        pa = [1/4, 1/4, 1/4, 1/4];
    
        % initialize the set of random solutions
        X = rand(npop, dim) .* (ub - lb) + lb;
        gbestsol = zeros(1, dim);
        gbestval = inf;
        objective_values = zeros(1, npop);
    
        % calculate the fitness of the first set and find the best one
        for i = 1 : npop
            objective_values(1,i) = myCost(X(i,:), minimumVelocity, maximumVelocity, minimumTheta, ...
                                maximumTheta, robotOld, dynamicOld, totalRobot, robotRadius, robotGoal, ...
                                totalStatic, staticRadius, static, totalDynamic, dynamicRadius);
            fitcount = fitcount + 1;
            if objective_values(1,i) < gbestval
                gbestsol = X(i,:);
                gbestval = objective_values(1,i);
            end
        end
    
        % roulette wheel selection
        for i = 1 : size(X,1)
            point_strateji(1,i) = RouletteWheelSelection(pa);
        end
    
        % main loop
        t = 1; 
        while fitcount <= maxit
    
            % Eq. (3.4) in SCA paper
            r1 = a - t * (a / maxit); % r1 decreases linearly from a to 0
    
            % update the position of solutions with respect to destination
            for i = 1 : npop % in i-th solution
    
                % selecting two different solutions in the swarm
                R1 = ceil(rand*size(X,1));
                R2 = ceil(rand*size(X,1));
                R3 = ceil(rand*size(X,1));
                while R1 == R2
                    R2 = ceil(rand*size(X,1));
                end
                while R1 == R3
                    R3 = ceil(rand*size(X,1));
                end
                while R2 == R3
                    R3 = ceil(rand*size(X,1));
                end
                while R1 == R2
                    R2 = ceil(rand*size(X,1));
                end
                L = rand;
    
                % strategy of the solution i
                kk = point_strateji(i);
                
                % strategy choice
                if kk == 1 % Eq. (2) in our paper
    
                    for j = 1 : dim % in j-th dimension
    
                        % Update r2, r3, and r4 for Eq. (3.3) in SCA paper
                        r2 = (2 * pi) * rand();
                        r3 = 2 * rand;
                        r4 = rand();
    
                        % Eq. (3.3) in SCA paper
                        if r4 < 0.5
                            % Eq. (3.1) in SCA paper
                            newX(i,j) = X(i,j) + (r1 * sin(r2) * abs(r3 * gbestsol(j) - X(i,j)));
                        else
                            % Eq. (3.2) in SCA paper
                            newX(i,j) = X(i,j) + (r1 * cos(r2) * abs(r3 * gbestsol(j) - X(i,j)));
                        end
    
                    end
    
                end
    
                if kk == 2 % Eq. (4) in our paper
    
                    for j = 1 : dim % in j-th dimension
                        
                        if rand <= CR
                            newX(i,j) = X(R1,j) + FF * ( X(R2,j) - X(R3,j) );
                        else
                            newX(i,j) = X(i,j);
                        end
    
                    end
    
                end
    
                if kk == 3 % Eq. (5) in our paper
    
                    for j = 1 : dim % in j-th dimension
    
                        if rand <= CR
                            newX(i,j) = X(i,j) + FF * ( gbestsol(j) - X(i,j) + X(R1,j) - X(R2,j) );
                        else
                            newX(i,j) = X(i,j);
                        end
    
                    end
    
                end
    
                if kk == 4 % Eq. (6) in our paper
    
                    for j = 1 : dim % in j-th dimension
    
                        newX(i,j) = X(i,j) + L * ( X(R1,j) - X(i,j) ) + FF * ( X(R2,j) - X(R3,j) );
    
                    end
    
                end
    
            end
    
            for i = 1 : npop
    
                % check if solutions go outside the search spaceand bring them back
                newX(i,:) = max(newX(i,:),lb);
                newX(i,:) = min(newX(i,:),ub);
    
                % calculate the objective values
                new_objective_value(i,:) = myCost(newX(i,:), minimumVelocity, maximumVelocity, minimumTheta, ...
                                maximumTheta, robotOld, dynamicOld, totalRobot, robotRadius, robotGoal, ...
                                totalStatic, staticRadius, static, totalDynamic, dynamicRadius);
                fitcount = fitcount + 1;
    
                % update the destination if there is a better solution
                % also, increase selection counter of the relevant strategy
                if new_objective_value(i,:) < objective_values(1,i)
                    X(i,:) = newX(i,:);
                    counter(i) = 0;
                    delta_fes(point_strateji(i)) = delta_fes(point_strateji(i)) + 1;
                    objective_values(1,i) = new_objective_value(i,:);
    
                    % update the global
                    if objective_values(1,i) < gbestval
                        gbestsol = X(i,:);
                        gbestval = objective_values(1,i);
                    end
    
                else
                    
                    % increase non-selection counter of the relevant strategy
                    counter(i) = counter(i) + 1;
                    
                end
    
            end
    
            % convergence
            convergence_curve(t) = gbestval;
    
            % increase the iteration counter
            t = t + 1;
    
            % calculate the probability of each strategy
            top = sum(delta_fes);
            for o = 1 : K
                pa(o) = delta_fes(o) / top;
            end   
    
            % roulette wheel selection
            for i = 1 : size(X,1)
                point_strateji(t,i) = RouletteWheelSelection(pa);
            end
    
    
        end
    
    % bestsol and bestcost
    bestSol = gbestsol;
    bestCost = gbestval;

end

