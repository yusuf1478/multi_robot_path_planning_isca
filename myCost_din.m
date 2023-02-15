function fitn = myCost_din(pop, problem_alt_limit, problem_ust_limit, robot_eski_x, robot_eski_y, robot_hedef_x, robot_hedef_y, robot_yaricap, engel_sayisi, engel_yaricap, ...
    engel_x, engel_y, robot_sayisi, din)
    
    
    for r = 1 : robot_sayisi
        
        pop(2*r) = interp1( [problem_alt_limit, problem_ust_limit], [0, 2*pi], pop(2*r) );
        robot_yeni_x(r) = robot_eski_x(r) + pop((2*r)-1) * cos(pop(2*r));
        robot_yeni_y(r) = robot_eski_y(r) + pop((2*r)-1) * sin(pop(2*r));

        % mesafe
        fit(r) = sqrt( ( robot_yeni_x(r) - robot_eski_x(r) )^2 + ( robot_yeni_y(r) - robot_eski_y(r) )^2 ) + sqrt( ( robot_yeni_x(r) - robot_hedef_x(r) )^2 + ...
                                                                                                                   ( robot_yeni_y(r) - robot_hedef_y(r) )^2 );
        % statik engel
        guvenlik_mesafesi = robot_yaricap(r);
        for j = 1 : engel_sayisi
            engele_olan_mesafe = sqrt( ( robot_yeni_x(r) - engel_x(j) )^2 + ( robot_yeni_y(r) - engel_y(j) )^2 );
            if engele_olan_mesafe < engel_yaricap(j) + robot_yaricap(r) + guvenlik_mesafesi
                fit(r) = fit(r) + 10000;
            end
        end
        
        % dinamik engel
        for j = 1 : din.engel_sayisi
            din_engele_olan_mesafe = sqrt( ( robot_yeni_x(r) - din.engel_x(j) )^2 + ( robot_yeni_y(r) - din.engel_y(j) )^2 );
            if din_engele_olan_mesafe < din.engel_yaricap(j) + robot_yaricap(r) + guvenlik_mesafesi
                fit(r) = fit(r) + 10000;
            end
        end

        % diğer robotlar
        ol_x = robot_eski_x;
        ol_y = robot_eski_y;
        ol_x(r) = [];
        ol_y(r) = [];
        for j = 1 : (robot_sayisi-1)
            robota_olan_mesafe = sqrt( ( robot_yeni_x(r) - ol_x(j) )^2 + ( robot_yeni_y(r) - ol_y(j) )^2 );
            if robota_olan_mesafe < robot_yaricap(r)*3
                fit(r) = fit(r) + 10000;
            end
        end

    end
    
     fitn = sum(fit);

end








%     for r = 1 : numRob
%         old = oldP(r,:);
%         xt = targetR(r,1);
%         yt = targetR(r,2);
%         pos(2*r) = interp1(InterpolX, InterpolY, pos(2*r));
%         
%         % mesafe amaç fonk. pos(1)=v, pos(2)=teta
%         newp(1) = old(1) + pos((2*r)-1) * cos(pos(2*r));
%         newp(2) = old(2) + pos((2*r)-1) * sin(pos(2*r));
%         fit(r) = sqrt( ( newp(1) - old(1) )^2 + ( newp(2) - old(2) )^2 ) + sqrt( ( newp(1) - xt(1) )^2 + ( newp(2) - yt(1) )^2 );
% 
%         engel amaç fonk.
%         guv = sizeR / 2;
%         for j = 1 : numel(obsx)
%             newpObs = sqrt( ( newp(1) - obsx(j) )^2 + ( newp(2) - obsy(j) )^2 );
%             if newpObs < obsr(j) + sizeR + guv
%                 fit(r) = fit(r) + 1000000;
%             end
%         end
%         
%         % diğer robotlar amaç fonk.
%         ol = oldP;
%         ol(r,:) = [];
%         for j = 1 : numRob-1
%             newpRob = sqrt( ( newp(1) - ol(j,1) )^2 + ( newp(2) - ol(j,2) )^2 );
%             if newpRob < sizeR*3
%                 fit(r) = fit(r) + 1000000;
%             end
%         end
%     end
%     
%     fitn = sum(fit);
    

