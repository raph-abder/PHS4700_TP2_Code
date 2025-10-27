    function [coup, vbf, t, x, y, z]= Devoir2 (option,xy0,vb0,wb0)
    % verifie la norme
    if norm(vb0) >= 100
        error('La norme de vb0 (%.2f m/s) dépasse la limite de 100 m/s.', norm(vb0));
    end

    % initialise les contantes
    C = constantes();
    fprintf('Temps interval de : %f \n', C.T_interval)

    % initialisation de l'etat q et la vitesse angulaire
    q0 = [vb0(1); vb0(2); vb0(3); xy0(1) ; xy0(2); C.R_B];
    wb0 = wb0(:);
    q = q0;

    idx = 1;
    t_cur = C.t_debut;

    % allouer l'espace memoire necessaire
    max_steps = round(C.t_max / C.T_interval) + 1;
    t = zeros(max_steps, 1);
    x = zeros(max_steps, 1);
    y = zeros(max_steps, 1);
    z = zeros(max_steps, 1);

    % initialiser les premieres donnees
    t(1) = C.t_debut;
    x(1) = q0(4);
    y(1) = q0(5);
    z(1) = q0(6);

    % regrouper les parametres
    params = {C, option, wb0};

    q_precedent = q;
    t_precedent = t_cur;

    while true
        % verifie si le temps c'est ecoule
        if t_cur >= C.t_max
            coup = 1;
            break
        end

        % resolution d'equation differentiel avec RK4
        qs = SEDRK4t0(q, t_cur, C.T_interval, @derive_q, params);

        % mise a jour du temps
        t_cur = t_cur + C.T_interval;
        idx = idx + 1;

        % mettre les elements necessaire en memoire
        t(idx) = t_cur;
        x(idx) = qs(4);
        y(idx) = qs(5);
        z(idx) = qs(6);


        % verifier la position
        if qs(6) <= C.R_B

            [t_impact, x_impact, y_impact, vbf_impact] = ...
                interpoler_impact(q_precedent, qs, t_precedent, t_cur, C.R_B);


            q = [vbf_impact(1); vbf_impact(2); vbf_impact(3);
                 x_impact; y_impact; C.R_B];


            % redefinir pour garder l'impact en fait
            x(idx) = q(4);
            y(idx) = q(5);
            z(idx) = q(6);
            t(idx) = t_impact ;


            if est_dans_coupe(C, q(4), q(5), q(6))
                coup = 0;
            else
                coup = 1;
            end
            break
        elseif est_dans_region_boisee(C, qs(4), qs(5))
            coup = 2;
            q = qs;
            break
        end

        q_precedent = q;
        t_precedent = t_cur - C.T_interval;
        q = qs;
    end

    % mettre les elements finaux en memoire
    vbf = q(1:3);
    t = t(1:idx);
    x = x(1:idx);
    y = y(1:idx);
    z = z(1:idx);
    fprintf('Final position: (%.3f, %.3f, %.3f)\n', x(idx), y(idx), z(idx));
end


% Constantes physiques
function C = constantes()
    C.M_B     = 45.9e-3;        %   masse balle
    C.R_B     = 21.35e-3;       %    rayon balle
    C.R_COUPE = 5.4e-2;         %    rayon coupe
    C.G       = 9.8;            %
    C.RHO     = 1.2;            %
    C.C_V     = 0.14;           %   coefficient de frottement visqueux
    C.V0_MAX  = 100;            %   vitesse initiale max

    C.A          = pi * C.R_B^2;      % aire
    C.G_VECTEUR  = [0; 0; -C.G];      % constante gravitationnel


    C.C_M_COEFF  = 0.000791;          % coeff empirique Magnus

    % Position de la coupe (ex. depuis ton code)
    C.X_COUPE = 150 - 8;
    C.Y_COUPE = 150 - 8;

    %  Région boisée (ex. depuis ton code)
    C.L_VERT_HAUTEUR = 150;
    C.L_VERT_LARGEUR = 30;
    C.L_HOR_LONGUEUR = 150;
    C.L_HOR_LARGEUR  = 20;
    C.T_interval = 0.0001;
    C.t_max = 100;
    C.t_debut = 0;
end


% helper pour verifier la position de la base
function isCollisionBoisee = est_dans_region_boisee(C, x, y)
    dansPartieVerticale = (x>=0) & (y>=0) & (x <= C.L_VERT_LARGEUR) & (y<= C.L_VERT_HAUTEUR);
    dansPartieHorizontale = (x>=0) & (y>= C.L_VERT_HAUTEUR - C.L_HOR_LARGEUR) & (x<= C.L_HOR_LONGUEUR) & (y <= C.L_VERT_HAUTEUR);
    isCollisionBoisee = ~(dansPartieVerticale | dansPartieHorizontale);
end

function isDansCoupe = est_dans_coupe(C, x, y, z)
    distance_centre = hypot(x - C.X_COUPE, y - C.Y_COUPE);
    isDansCoupe = (z <= C.R_B) & (distance_centre <= C.R_COUPE);
end


% ---- FORCE ------
function Fg = force_gravite(C)
    Fg = C.M_B * C.G_VECTEUR;
end

function Fv = force_visqueuse(C, V)
    % calcul de la norme
    vnorm = norm(V);
    Fv = -(1/2) * C.RHO * C.C_V * C.A * vnorm * V; %
end

function FM = force_magnus(C, V, wb)
    % calcul des attributs de la fonction
    CM   = C.C_M_COEFF * norm(wb);
    V2   = norm(V)^2;
    wxc  = cross(wb, V);
    nx   = norm(wxc);
    % eviter le division par 0
    if nx > 1e-10
        FM = 0.5 * C.RHO * CM * C.A * V2 * (wxc / nx);
    else
        FM = [0; 0; 0];
    end
end

% ---- Acceleration ------

function a = accelerationSelonOption(C,option,V,W)
    switch option
        case 1
            a = C.G_VECTEUR;
        case 2   % gravité + frottement visqueux
            a = acc_gravite_visqueux(C,V);
        case 3   % gravité + visqueux + Magnus
            a = acc_gravite_visqueux_magnus(C,V,W);
        otherwise
            error('option doit être 1, 2 ou 3');
    end
end

% option #2
function a = acc_gravite_visqueux(C, V)
    Fg = force_gravite(C);
    Fv = force_visqueuse(C,V);
    a = (Fg + Fv) / C.M_B;
end

% option #3
function a = acc_gravite_visqueux_magnus(C,V,WB)
    Fg = force_gravite(C);
    Fv = force_visqueuse(C,V);
    Fm = force_magnus(C ,V, WB);
    a = (Fg + Fv + Fm) / C.M_B;
end


% ---- RK4 et help RK4 ------
% code venant du professeur et accepte de pouvoir prendre
function qs=SEDRK4t0(q0,t0,Deltat,derive_q,params)
    % Equation a resoudre : dq/dt=g(q,t)
    %   qs        : solution [q(to+Deltat)]
    %   q0        : conditions initiales [q(t0)]
    %   Deltat    : intervalle de temps
    %   g         : membre de droite de ED.
    k1=feval(derive_q,q0,t0,params);
    k2=feval(derive_q,q0+k1*Deltat/2,t0+Deltat/2,params);
    k3=feval(derive_q,q0+k2*Deltat/2,t0+Deltat/2,params);
    k4=feval(derive_q,q0+k3*Deltat,t0+Deltat,params);
    qs=q0+Deltat*(k1+2*k2+2*k3+k4)/6;
end

% application pour les k1,k2,k3,k4
function q_derive = derive_q(q0, t, params)
    C      = params{1};
    option = params{2};
    wb0    = params{3};
    V = q0(1:3);
    a = accelerationSelonOption(C, option, V, wb0);
    q_derive = [a(1); a(2); a(3); q0(1); q0(2); q0(3)];
end

function [t_impact, x_impact, y_impact, vbf_impact] = interpoler_impact(q_avant, q_apres, t_avant, t_apres, z_sol)

    z_avant = q_avant(6);
    z_apres = q_apres(6);

    % Vérification que l'impact a bien eu lieu
    if z_avant <= z_sol || z_apres >= z_sol
        error('Pas d''impact détecté entre les deux états fournis');
    end

    % Interpolation linéaire pour trouver le temps exact où z = 0
    taux = (z_sol - z_avant) / (z_apres - z_avant);
    t_impact = t_avant + taux * (t_apres - t_avant);

    % Interpolation linéaire de la position
    x_impact = q_avant(4) + taux * (q_apres(4) - q_avant(4));
    y_impact = q_avant(5) + taux * (q_apres(5) - q_avant(5));

    % Interpolation linéaire de la vitesse
    vx_impact = q_avant(1) + taux * (q_apres(1) - q_avant(1));
    vy_impact = q_avant(2) + taux * (q_apres(2) - q_avant(2));
    vz_impact = q_avant(3) + taux * (q_apres(3) - q_avant(3));

    vbf_impact = [vx_impact; vy_impact; vz_impact];
end



