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

    % Position de la coupe (définie selon la figure)
    C.X_COUPE = 150 - 8;              % [m]
    C.Y_COUPE = 130 + 8;              % [m]

    %  Région du terrain (green)
    C.L_VERT_HAUTEUR = 150;
    C.L_VERT_LARGEUR = 30;
    C.L_HOR_LONGUEUR = 150;
    C.L_HOR_LARGEUR  = 20;
end
