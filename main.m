
clc;
clear;
close all;

runc = 1;
costf = @myCost_din;
hed_mycell = cell(runc,1);
% step = 0;

for ir = 1 : runc
    
%     tic;
    close all;
    fprintf('run: %d\n',ir);
    
%     if step

    % senaryo 1
    engel_x = [10 25 25 40 35 75 85];
    engel_y = [80 90 30 65 25 40 15];
    engel_yaricap = [4 3 3 5 4 5 3];
    engel_sayisi = numel(engel_x);

    robot_baslama_x = [5 25 45 95 85 95];
    robot_baslama_y = [95 10 75 95 40 5];
    robot_hedef_x =   [40 25 10 65 40 65];
    robot_hedef_y =   [13 97 55 15 40 75];

%     robot_baslama_x = [5];
%     robot_baslama_y = [95];
%     robot_hedef_x = [40];
%     robot_hedef_y = [13];

%     robot_baslama_x = [5 25];
%     robot_baslama_y = [95 10];
%     robot_hedef_x = [40 25];
%     robot_hedef_y = [13 97];
% 
%     robot_baslama_x = [5 25 45];
%     robot_baslama_y = [95 10 75];
%     robot_hedef_x = [40 25 10];
%     robot_hedef_y = [13 97 55];

%     robot_baslama_x = [5 25 45 95];
%     robot_baslama_y = [95 10 75 95];
%     robot_hedef_x = [40 25 10 65];
%     robot_hedef_y = [13 97 55 15];

%     robot_baslama_x = [5 25 45 95 85];
%     robot_baslama_y = [95 10 75 95 40];
%     robot_hedef_x = [40 25 10 65 40];
%     robot_hedef_y = [13 97 55 15 40];

    din.engel_baslama_x = [15 55 75];
    din.engel_baslama_y = [40 20 90];
    din.engel_hedef_x = [35 50 95];
    din.engel_hedef_y = [85 50 75];
    din.engel_sayisi = numel(din.engel_baslama_x);
    din.engel_yaricap = repmat(1.5,1,din.engel_sayisi);
    din.vel_ob = [0.5 0.45 1.2];
    boy = 100;
    
    % senaryo 2
%     engel_x = [20 20 40 50 50 70 70 70 110 140 140 160 180 180];
%     engel_y = [80 160 60 60 90 20 110 150 100 160 175 90 100 170];
%     engel_yaricap = [4 4 3 4 3 3 3 3 5 5 3 3 3 3];
%     engel_sayisi = numel(engel_x);
%     robot_baslama_x = [10  20  50  60  70  90  110  110 110 120 180  180];
%     robot_baslama_y = [60  190 150 60  170 120  60  160 180 120  80  160];
%     robot_hedef_x = [30 20 50 30 70 140 190 150 160 60 180 180];
%     robot_hedef_y = [90 40 30 60 90 50  110 180 140 10 150 180];
%     din.engel_baslama_x = [10  40 60  80 120 170];
%     din.engel_baslama_y = [150 90 160 85 50  120];
%     din.engel_hedef_x = [30  60  80  100 140 190];
%     din.engel_hedef_y = [100 130 120 30  100 125];
%     din.engel_sayisi = numel(din.engel_baslama_x);
%     din.engel_yaricap = repmat(1.5,1,din.engel_sayisi);
%     din.vel_ob = [0.5 0.5 0.6 0.3 0.4 0.25];
%     boy = 200;

    robot_sayisi = numel(robot_baslama_x);
    robot_yaricap = repmat(1,1,robot_sayisi);
    d_adim_sayisi = zeros(1,robot_sayisi);
    d_yol_uzunlugu = zeros(1,robot_sayisi);
 
    theta = linspace(0, 2*pi, 100);
    for i = 1 : engel_sayisi % statik engel
        plt2 = fill( engel_x(i) + engel_yaricap(i) * cos(theta), engel_y(i) + engel_yaricap(i) * sin(theta), [0.5 0.5 0.5] );
        hold on;
    end
    for i = 1 : din.engel_sayisi % dianamik engel ve dianamik engel çizgisi
        plt2(i) = fill( din.engel_baslama_x(i) + din.engel_yaricap(i) * cos(theta), din.engel_baslama_y(i) + din.engel_yaricap(i) * sin(theta), [0.1 0.2 0.9] );
        line([din.engel_baslama_x(i) din.engel_hedef_x(i)],[din.engel_baslama_y(i) din.engel_hedef_y(i)],'Color','blue','LineStyle','--'), box on;
        plot(din.engel_hedef_x(i),din.engel_hedef_y(i),'x','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',10,'LineWidth',2);
    end
    for i = 1 : robot_sayisi % robot ve robot çizgisi
        plt1(i) = fill( robot_baslama_x(i) + robot_yaricap(i) * cos(theta), robot_baslama_y(i) + robot_yaricap(i) * sin(theta), [0.5 0.7 0.8] );
        line([robot_baslama_x(i) robot_hedef_x(i)],[robot_baslama_y(i) robot_hedef_y(i)],'Color','black','LineStyle','--'), box on;
        e4 = plot(robot_hedef_x(i),robot_hedef_y(i),'x','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',10,'LineWidth',2);
    end
    axis equal;
    axis([-5 boy+5 0 boy]);
    grid on;
    pause(1);
     
    % her robotun ideal yol mesafesi (düz çizgi olan, engelleri kesen)
    for r = 1 : robot_sayisi
        ideal_mes(r) = sqrt( ( robot_baslama_x(r) - robot_hedef_x(r) )^2 + ( robot_baslama_y(r) - robot_hedef_y(r) )^2 );
    end

    maksimum_fe_sayisi = 1000;  % 500, 0.1-2
    populasyon_sayisi = 30;
    problem_boyutu = 2 * robot_sayisi;
    problem_alt_limit = 1;
    problem_ust_limit = 1.5;

    step = 1;
    robot_eski_x = robot_baslama_x;
    robot_eski_y = robot_baslama_y;
    robot_yol_x = [robot_eski_x];
    robot_yol_y = [robot_eski_y];
    din.engel_x = din.engel_baslama_x;
    din.engel_y = din.engel_baslama_y;
    din_yol_x = [din.engel_x];
    din_yol_y = [din.engel_y];

    for r = 1 : robot_sayisi
        hedefe_olan_mesafe(r) = sqrt( ( robot_eski_x(r) - robot_hedef_x(r) )^2 + ( robot_eski_y(r) - robot_hedef_y(r) )^2 );
    end
    [a, ind] = max(hedefe_olan_mesafe);
    durdurma_kriteri_mesafesi = hedefe_olan_mesafe(ind);
    
    tic;

    while durdurma_kriteri_mesafesi > 1 && toc < 338 % aoa için 338

        delete(plt1);
        delete(plt2);

        % statik
        [en_iyi_cozum, en_iyi_maliyet] = isca_f(maksimum_fe_sayisi, populasyon_sayisi, problem_boyutu, problem_ust_limit, problem_alt_limit, robot_eski_x, robot_eski_y, ...
            robot_hedef_x, robot_hedef_y, robot_yaricap, engel_sayisi, engel_yaricap, engel_x, engel_y, robot_sayisi, costf, din);
%         [en_iyi_cozum, en_iyi_maliyet] = sca_f(maksimum_fe_sayisi, populasyon_sayisi, problem_boyutu, problem_ust_limit, problem_alt_limit, robot_eski_x, robot_eski_y, ...
%             robot_hedef_x, robot_hedef_y, robot_yaricap, engel_sayisi, engel_yaricap, engel_x, engel_y, robot_sayisi, costf, din);

        maliyet(ir,step) = en_iyi_maliyet;
%         curv(step,:,ir) = cur;
        
%         for i = 1 : engel_sayisi % statik engel
%            fill( engel_x(i) + engel_yaricap(i) * cos(theta), engel_y(i) + engel_yaricap(i) * sin(theta), [0.5 0.5 0.5] );
%         end
        for r = 1 : robot_sayisi % robot hareketi
            hedefe_olan_mesafe(r) = sqrt( ( robot_eski_x(r) - robot_hedef_x(r) )^2 + ( robot_eski_y(r) - robot_hedef_y(r) )^2 );
            en_iyi_cozum(2*r) = interp1( [problem_alt_limit, problem_ust_limit], [0, 2*pi], en_iyi_cozum(2*r) );
            if hedefe_olan_mesafe(r) > 1    
                robot_yeni_x(r) = robot_eski_x(r) + en_iyi_cozum((2*r)-1) * cos(en_iyi_cozum(2*r));
                robot_yeni_y(r) = robot_eski_y(r) + en_iyi_cozum((2*r)-1) * sin(en_iyi_cozum(2*r));
                d_yol_uzunlugu(r) = d_yol_uzunlugu(r) + sqrt( ( robot_yeni_x(r) - robot_eski_x(r) )^2 + ( robot_yeni_y(r) - robot_eski_y(r) )^2 );
                d_adim_sayisi(r) = d_adim_sayisi(r) + 1;
                hed(d_adim_sayisi(r),r) = sqrt( ( robot_yeni_x(r) - robot_hedef_x(r) )^2 + ( robot_yeni_y(r) - robot_hedef_y(r) )^2 );
            end
            plt1(r) = fill( robot_yeni_x(r) + robot_yaricap(r) * cos(theta), robot_yeni_y(r) + robot_yaricap(r) * sin(theta), [0.5 0.7 0.8] );     
            line([robot_baslama_x(r) robot_hedef_x(r)],[robot_baslama_y(r) robot_hedef_y(r)],'Color','black','LineStyle','--'), box on;
            line([robot_eski_x(r) robot_yeni_x(r)],[robot_eski_y(r) robot_yeni_y(r)],'Color','red','LineStyle','--');
            
        end
        
        for i = 1 : din.engel_sayisi % dinamik engel hareketi
            din_hedefe_olan_mesafe(i) = sqrt( ( din.engel_x(i) - din.engel_hedef_x(i) )^2 + ( din.engel_y(i) - din.engel_hedef_y(i) )^2 );
            if din_hedefe_olan_mesafe(i) > 1
                egim(i) = atan2( din.engel_hedef_y(i) - din.engel_y(i) , din.engel_hedef_x(i) - din.engel_x(i) );
                din.engel_x(i) = din.engel_x(i) + din.vel_ob(i) * cos(egim(i));
                din.engel_y(i) = din.engel_y(i) + din.vel_ob(i) * sin(egim(i));
            end
        end
        for i = 1 : din.engel_sayisi % dinamik engel
            plt2(i) = fill( din.engel_x(i) + din.engel_yaricap(i) * cos(theta), din.engel_y(i) + din.engel_yaricap(i) * sin(theta), [0.1 0.2 0.9] );
            plot(din.engel_hedef_x(i),din.engel_hedef_y(i),'x','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',10,'LineWidth',2);
        end
% 
        pause(0.01);
        robot_eski_x = robot_yeni_x;
        robot_eski_y = robot_yeni_y;  
        robot_yol_x = [robot_yol_x; robot_yeni_x];
        robot_yol_y = [robot_yol_y; robot_yeni_y];
        din_yol_x = [din_yol_x; din.engel_x];
        din_yol_y = [din_yol_y; din.engel_y];

        for r = 1 : robot_sayisi
            hedefe_olan_mesafe(r) = sqrt( ( robot_eski_x(r) - robot_hedef_x(r) )^2 + ( robot_eski_y(r) - robot_hedef_y(r) )^2 );
        end
        [a, ind] = max(hedefe_olan_mesafe);
        durdurma_kriteri_mesafesi = hedefe_olan_mesafe(ind);
        
%          zam(ir,step) = toc;
         
        step = step + 1;
       

    end
    
    eval(['yolx' num2str(ir) '= robot_yol_x']);
    eval(['yoly' num2str(ir) '= robot_yol_y']);
%     yolx(:,:,ir) = robot_yol_x;
%     yoly(:,:,ir) = robot_yol_y;
    mes(ir,:)=d_yol_uzunlugu;
    adi(ir,:)=d_adim_sayisi;
    hed_s = sum(hed,2)';
    hed_mycell{ir} = hed_s; % grafik için
%     hed_mycell{ir} = hed; % grafik için
    hed_ss(ir) = sum(hed_s); % tablo için
%     hed_ss(ir,:) = hed_s;
%     hed_s(:,:,ir) = hed_ss;
    sapma(ir) = sum(d_yol_uzunlugu - ideal_mes);
    zam(ir) = toc;

end


for i = 1 : size(zam,2)
    topzam(i) = sum(zam(1:i));
end


% karşılaştırma için: hed_s, sapma, sonuc_mesafe, sonuc_adim, zam
for roo = 1 : robot_sayisi
    sonuc_mesafe(roo) = mean(mes(:,roo));
end
for roo = 1 : robot_sayisi
    sonuc_adim(roo) = round(mean(adi(:,roo)));
end

so1 = mean(sapma);
so2 = mean(hed_ss);
so3 = sum(mean(maliyet));
so4 = mean(zam);





