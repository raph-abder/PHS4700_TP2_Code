function C = constantes()
    % Constantes physiques
    C.M_B     = 45.9e-3;        % [kg]  masse balle
    C.R_B     = 21.35e-3;       % [m]   rayon balle
    C.R_COUPE = 5.4e-2;         % [m]   rayon coupe
    C.G       = 9.8;            % [m/s^2]
    C.RHO     = 1.2;            % [kg/m^3]
    C.C_V     = 0.14;           % [-]   coeff frottement visqueux
    C.V0_MAX  = 100;            % [m/s] vitesse initiale max

    % Constantes dérivées
    C.A          = pi * C.R_B^2;      % [m^2] aire
    C.G_VECTEUR  = [0; 0; -C.G];      % [m/s^2]

    % Magnus
    C.C_M_COEFF  = 0.000791;          % [-]    coeff empirique Magnus

    % Position de la coupe (ex. depuis ton code)
    C.X_COUPE = 150 - 8;              % [m]
    C.Y_COUPE = 130 + 8;              % [m]

    %  Région boisée (ex. depuis ton code)
    C.L_VERT_HAUTEUR = 150;
    C.L_VERT_LARGEUR = 30;
    C.L_HOR_LONGUEUR = 150;
    C.L_HOR_LARGEUR  = 20;
end


function isCollisionBoisee = est_dans_region_boisee(C, x, y)
    dansPartieVerticale = (x>=0) & (y>=0) & (x <= C.L_VERT_LARGEUR) & (y<= C.L_VERT_HAUTEUR);
    dansPartieHorizontale = (x>=0) & (y>= C.L_VERT_HAUTEUR - C.L_HOR_LARGEUR) & (x<= C.L_HOR_LONGUEUR) & (y <= C.L_VERT_HAUTEUR);
    isCollisionBoisee = ~(dansPartieVerticale | dansPartieHorizontale);
end 

function isDansCoupe = est_dans_coupe(C, x, y, z)
    distance_centre = hypot(x - C.X_COUPE, y - C.Y_COUPE);
    isDansCoupe = (z <= 0) & (distance_centre <= C.R_COUPE);
end

function Fg = force_gravite(C, M) %Prends la masse
    Fg = M * C.G_VECTEUR; % [N]
end

function Fv = force_visqueuse(C, V)
    % Frottement visqueux : Fv = - 1/2 * rho * C_V * A * |v| * v
    vnorm = norm(V);
    Fv = -(1/2) * C.RHO * C_V * A * vnorm * V; % [N]
end

function FM = force_magnus(C, V, wb)
% FM = 1/2 * rho * C_M(|ω|) * A * |v|^2 * ( (ω x v) / |ω x v| )
% où C_M(|ω|) = C.C_M_COEFF * |ω|
    CM   = C.C_M_COEFF * norm(wb);
    V2   = norm(V)^2;
    wxc  = cross(wb, V);
    nx   = norm(wxc);
    FM = 0.5 * C.RHO * CM * C.A * V2 * (wxc / nx);   % [N]
end


function a = acc_gravite_visqueux(C, V)
    Fg = force_gravite(C.M_B);
    Fv = force_visqueuse(V, C);
    a = (Fg + Fv) / C.M_B;
end

function a = acc_gravite_visqueux_magnus(C,WB,V)
    Fg = force_gravite(C.M_B);
    Fv = force_visqueuse(V, C);
    Fm = force_magnus(V, WB, C);
    a = (Fg + Fv + Fm) / C.M_B;
end


function a = accelerationSelonOption(C,option)
    switch option
        case 1
            a = C.G_VECTEUR
        case 2   % gravité + frottement visqueux
            a = acc_gravite_visqueux(C,vb0)
        case 3   % gravité + visqueux + Magnus
            a = acc_gravite_visqueux_magnus(C,wb0,vb0)
        otherwise
            error('option doit être 1, 2 ou 3');
    end
end


% FONCTION DU DEVOIR 
function [coup vbf t x y z]= Devoir2 (option,xy0,vb0,wb0)

    C = constantes();
    a = accelerationSelonOption(C, option)

end