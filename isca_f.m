function [en_iyi_cozum, en_iyi_maliyet] = isca_f(maksimum_fe_sayisi, populasyon_sayisi, problem_boyutu, problem_ust_limit, problem_alt_limit, robot_eski_x, robot_eski_y, ...
                                                    robot_hedef_x, robot_hedef_y, robot_yaricap, engel_sayisi, engel_yaricap, engel_x, engel_y, robot_sayisi,costf,din)
    
      % kontrol parametreleri
    npop = populasyon_sayisi;
    dim = problem_boyutu;
    maxit = maksimum_fe_sayisi; 
    lb = problem_alt_limit;
    ub = problem_ust_limit;
     myCost=costf;

     % iyileştirme
    counter = zeros(1, npop);
    K = 4; % strateji sayısı
    % delta_f = zeros(1,K);
    delta_fes = zeros(1,K);
    % delta_a_f = zeros(1,K);
    pa = [1/4, 1/4, 1/4, 1/4];

    % iyileştirme - ek kontrol parametreleri
    pa_min = 0.1;
    FF = 0.8;
    CR = 0.95;


%             tic;
            fitcount = 0;

            % initialization
            X = rand(npop, dim) .* (ub - lb) + lb;
            gbestsol = zeros(1, dim);
            gbestval = inf;
            objective_values = zeros(1, npop);


            % Calculate the fitness of the first set and find the best one
            for i = 1 : npop

                objective_values(1,i) = myCost(X(i,:),lb, ub, robot_eski_x, robot_eski_y, robot_hedef_x, robot_hedef_y, robot_yaricap, ...
                                                engel_sayisi, engel_yaricap, engel_x, engel_y, robot_sayisi,din);
                fitcount = fitcount + 1;
                if objective_values(1,i) < gbestval
                    gbestsol = X(i,:);
                    gbestval = objective_values(1,i);
                end

            end

            % iyileştirme (rulet seçimi)
            for i = 1 : size(X,1)
                point_strateji(1,i) = RouletteWheelSelection(pa);
            end

            % iterasyon sayacı
            t = 1; 

            % Main loop
            while fitcount <= maxit

                % Eq. (3.4)
                a = 2;
                maxit = maxit;
                r1 = a - t * ((a) / maxit); % r1 decreases linearly from a to 0

                % Update the position of solutions with respect to destination
                for i = 1 : npop % in i-th solution

                    % iyileştirme - selecting two different points in the group
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

                    % iyileştirme - i. çözümün stratejisi
                    kk = point_strateji(i);

                    if kk == 1

                        for j = 1 : dim % in j-th dimension

                            % Update r2, r3, and r4 for Eq. (3.3)
                            r2 = (2 * pi) * rand();
                            r3 = 2 * rand;
                            r4 = rand();

                            % Eq. (3.3)
                            if r4 < 0.5
                                % Eq. (3.1)
                                newX(i,j) = X(i,j) + (r1 * sin(r2) * abs(r3 * gbestsol(j) - X(i,j)));
                            else
                                % Eq. (3.2)
                                newX(i,j) = X(i,j) + (r1 * cos(r2) * abs(r3 * gbestsol(j) - X(i,j)));
                            end

                        end

                    end

                    if kk == 2

                        for j = 1 : dim % in j-th dimension

                            % Update r2, r3, and r4 for Eq. (3.3)
    %                         r2 = (2 * pi) * rand();
    %                         r3 = 2 * rand;
    %                         r4 = rand();

                            if rand <= CR
                                newX(i,j) = X(R1,j) + FF * ( X(R2,j) - X(R3,j) );
                            else
                                newX(i,j) = X(i,j);
                            end

                        end

                    end

                    if kk == 3

                        for j = 1 : dim % in j-th dimension

                            % Update r2, r3, and r4 for Eq. (3.3)
    %                         r2 = (2 * pi) * rand();
    %                         r3 = 2 * rand;
    %                         r4 = rand();

                            if rand <= CR
                                newX(i,j) = X(i,j) + FF * ( gbestsol(j) - X(i,j) + X(R1,j) - X(R2,j) );
                            else
                                newX(i,j) = X(i,j);
                            end

                        end

                    end

                    if kk == 4

                        for j = 1 : dim % in j-th dimension

                            % Update r2, r3, and r4 for Eq. (3.3)
    %                         r2 = (2 * pi) * rand();
    %                         r3 = 2 * rand;
    %                         r4 = rand();

                            newX(i,j) = X(i,j) + L * ( X(R1,j) - X(i,j) ) + FF * ( X(R2,j) - X(R3,j) );

                        end

                    end

                end

                for i = 1 : npop

                    % Check if solutions go outside the search spaceand bring them back
                    newX(i,:) = max(newX(i,:),lb);
                    newX(i,:) = min(newX(i,:),ub);
    %                 Flag4ub = newX(i,:) > ub;
    %                 Flag4lb = newX(i,:) < lb;
    %                 newX(i,:) = (newX(i,:) .* (~(Flag4ub+Flag4lb))) + ub .* Flag4ub + lb .* Flag4lb;

                    % Calculate the objective values
    %                 objective_values(1,i) = fobj(X(i,:));
                    new_objective_value(i,:) = myCost(newX(i,:),lb, ub, robot_eski_x, robot_eski_y, robot_hedef_x, robot_hedef_y, robot_yaricap, ...
                                                engel_sayisi, engel_yaricap, engel_x, engel_y, robot_sayisi,din);
                    fitcount = fitcount + 1;

                    % iyileştirme
                    if new_objective_value(i,:) < objective_values(1,i)
                        X(i,:) = newX(i,:);
                        counter(i) = 0;
    %                     delta_f(point_strateji(i)) = objective_values(1,i) + ( objective_values(1,i) - new_objective_value(i,:) );
                        delta_fes(point_strateji(i)) = delta_fes(point_strateji(i)) + 1;

                        objective_values(1,i) = new_objective_value(i,:);
                        % Update the global
                        if objective_values(1,i) < gbestval
                            gbestsol = X(i,:);
                            gbestval = objective_values(1,i);
                        end

                    else
                        counter(i) = counter(i) + 1;
                    end

                end

                % yakınsama
    %             convergence_curve(t) = gbestval;

                % Display the iteration and best optimum obtained so far
    %             fprintf('func=%d run=%d FE=%d ObjVal=%g\n',funCounter,runCounter,fitcount,gbestval);

                % Increase the iteration counter
                t = t + 1;

                % iyileştirme (her stratejinin delta_a_f değeri hesaplanır, 44. satır)
    %             for o = 1 : K
    %                 delta_a_f(o) = delta_f(o) / delta_fes(o);
    %             end

                % iyileştirme (sum (45. satır))
    %             toplam = sum(delta_a_f); 

                % iyileştirme (her stratejinin olasılığı hesaplanır)
                top = sum(delta_fes);
                for o = 1 : K
                    pa(o) = delta_fes(o) / top;
                end   

    %              % iyileştirme (rulet seçimi)
                for i = 1 : size(X,1)
                    point_strateji(t,i) = RouletteWheelSelection(pa);
                end

    %             for u = 1 : K
    %                 delta_f(u) = 0;
    %                 delta_fes(u) = 0;
    %             end



            end

        en_iyi_cozum = gbestsol;
        en_iyi_maliyet = gbestval;

end

