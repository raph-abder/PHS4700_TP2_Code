
% --- Constantes physiques ---
M_B = 45.9e-3;              % [kg]  Masse de la balle de golf (45.9 g)
R_B = 21.35e-3;             % [m]   Rayon de la balle de golf (21.35 mm)
R_COUPE = 5.4e-2;           % [m]   Rayon de la coupe (trou) (5.4 cm)
G = 9.8;                    % [m/s^2] Accélération gravitationnelle
RHO = 1.2;                  % [kg/m^3] Masse volumique de l'air
C_V = 0.14;                 % [-]  Coefficient de frottement visqueux
V0_MAX = 100;               % [m/s] Vitesse initiale maximale permise

% --- Constantes dérivées ---
A = pi * R_B^2;             % [m^2] Aire effective de la balle

% --- Coefficient du phénomène de Magnus ---
C_M_COEFF = 0.000791;       % [-] Constante empirique du coefficient de Magnus

% --- Vecteurs utiles ---
G_VECTEUR = [0, 0, -G];     % [m/s^2] Vecteur de l'accélération gravitationnelle



function isCollisionBoisee = est_dans_region_boisee(x, y)
    L_VERT_HAUTEUR = 150;   % hauteur de la bande verticale gauche
    L_VERT_LARGEUR = 30;    % largeur  de la bande verticale gauche
    L_HOR_LONGUEUR = 150;  % longueur de la bande horizontale haute
    L_HOR_LARGEUR = 20;    % largeur  de la bande horizontale haute
    
    dansPartieVerticale = (x>=0) & (y>=0) & (x <= L_VERT_LARGEUR) & (y<= L_VERT_HAUTEUR);
    dansPartieHorizontale = (x>=0) & (y>=L_VERT_HAUTEUR - L_HOR_LARGEUR) & (x<= L_HOR_LONGUEUR) & (y <= L_VERT_HAUTEUR);

    isCollisionBoisee = ~(dansPartieVerticale | dansPartieHorizontale);
end 

function isDansCoupe = est_dans_coupe(x, y, z)
    R_COUPE = 5.4e-2;
    X_COUPE = 150 - 8;
    Y_COUPE = 130 + 8;

    distance_centre = hypot(x - X_COUPE, y - Y_COUPE);
    isDansCoupe = (z <= 0) & (distance_centre <= R_COUPE);
end

function Fg = force_gravite(M) %Prends la masse
    % Poids : Fg = m * g
    Fg = M * G_VECTEUR; % [N]
end

function Fv = force_visqueuse(v) % prends un vecteur v : [vx; vy; vz] (m/s)
% Frottement visqueux : Fv = - 1/2 * rho * C_V * A * |v| * v
    vnorm = norm(v);
    Fv = -0.5 * RHO * C_V * A * vnorm * v; % [N]
end

function FM = force_magnus(v, wb, C) % où v : vitesse [3x1], wb : vitesse angulaire [3x1]
% FM = 1/2 * rho * C_M(|ω|) * A * |v|^2 * ( (ω x v) / |ω x v| )
% où C_M(|ω|) = C_M_COEFF * |ω|
    CM   = C_M_COEFF * norm(wb);
    v2   = norm(v)^2;
    wxc  = cross(wb, v);
    nx   = norm(wxc);

    FM = 0.5 * C.RHO * CM * C.A * v2 * (wxc / nx);   % [N]
end




% FONCTION DU DEVOIR 
function [coup vbf t x y z]= Devoir2 (option,xy0,vb0,wb0)

    % ==== SWITCH: choix du membre de droite dq/dt = g(q,t) ====
    switch option
        case 1   % gravité seule

        case 2   % gravité + frottement visqueux

        case 3   % gravité + visqueux + Magnus

        otherwise
            error('option doit être 1, 2 ou 3');
    end

end