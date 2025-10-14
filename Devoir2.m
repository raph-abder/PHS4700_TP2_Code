
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

