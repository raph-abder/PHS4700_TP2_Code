% =========================================================================
% SCRIPT POUR SIMULER ET VISUALISER LES TRAJECTOIRES DE GOLF
% =========================================================================
% Ce script exécute les 12 simulations demandées dans le Devoir 2 et
% génère un graphique pour chacun des 4 coups, comparant les 3 options
% de simulation (gravité seule, avec frottement, avec Magnus).
% =========================================================================

% Nettoyage de l'environnement
clear;
clc;
close all; % Ferme toutes les figures existantes

% --- Définition des données initiales pour les 4 coups ---
% Les données sont extraites du Tableau 1 du document.
% Nous utilisons des "cell arrays" pour stocker les vecteurs de chaque coup.

all_xy0 = {
    [13.10857; 142],      % Coup 1
    [15; 120],            % Coup 2
    [13.6; 130.766],      % Coup 3
    [13.3; 130]           % Coup 4
};

all_vb0 = {
    [30; 0; 21.052266],                 % Coup 1
    [25.3132; 20.3132; 22.052266],     % Coup 2
    [29.885; 2.6146; 21.052266],       % Coup 3
    [29.885; 2.6146; 21.052266]        % Coup 4
};

all_wb0 = {
    [0; -450; 0],                       % Coup 1
    [170; -170; -420],                  % Coup 2
    [33.16095; -379.068156; 0],         % Coup 3
    [0; -100; -100]                     % Coup 4
};

% Couleurs et options pour les graphiques
options = [1, 2, 3];
colors = {'r', 'b', 'k'}; % Rouge pour Option 1, Bleu pour 2, Noir pour 3
legend_entries = {'Option 1: Gravité', 'Option 2: Gravité + Frottement', 'Option 3: Gravité + Frottement + Magnus'};

% Récupération des constantes pour dessiner le terrain
C = constantes();

% --- Boucle principale pour simuler et tracer chaque coup ---
for i = 1:length(all_xy0)
    
    % Créer une nouvelle figure pour le coup actuel
    figure('Name', ['Trajectoire pour le Coup ' num2str(i)], 'NumberTitle', 'off');
    hold on; % Permet de superposer plusieurs tracés sur le même graphique
    
    % --- Dessin du terrain de golf ---
    % Définition des sommets du terrain en forme de L
    terrain_x = [0, C.L_VERT_LARGEUR, C.L_VERT_LARGEUR, C.L_HOR_LONGUEUR, C.L_HOR_LONGUEUR, 0, 0];
    terrain_y = [0, 0, C.L_VERT_HAUTEUR - C.L_HOR_LARGEUR, C.L_VERT_HAUTEUR - C.L_HOR_LARGEUR, C.L_VERT_HAUTEUR, C.L_VERT_HAUTEUR, 0];
    terrain_z = zeros(size(terrain_x)); % Le terrain est au niveau du sol (z=0)
    
    % Dessin de la surface verte avec la fonction 'patch'
    patch(terrain_x, terrain_y, terrain_z, [0.4, 0.8, 0.4]); % Couleur verte
    
    % --- Dessin de la coupe (avec un rayon visuel agrandi) ---
    % On définit un rayon pour l'affichage qui est plus grand que le rayon réel.
    % Changez la valeur '50' pour l'agrandir ou le rétrécir comme vous le souhaitez.
    rayon_visuel = C.R_COUPE * 20; % 20x plus grand pour être bien visible
    
    theta = linspace(0, 2*pi, 100);
    coupe_x = C.X_COUPE + rayon_visuel * cos(theta); % Utilise le rayon visuel
    coupe_y = C.Y_COUPE + rayon_visuel * sin(theta); % Utilise le rayon visuel
    coupe_z = zeros(size(theta)) + 0.01; % Légèrement au-dessus pour la visibilité
    patch(coupe_x, coupe_y, coupe_z, 'w', 'EdgeColor', 'k', 'LineWidth', 0.5); % Cercle blanc
    
    % --- Simulation et tracé des trajectoires pour les 3 options ---
    xy0 = all_xy0{i};
    vb0 = all_vb0{i};
    wb0 = all_wb0{i};
    
    % Array to store the handles of the plots we want in the legend
    plot_handles = []; 
    
    % Marquer le point de départ et CAPTURER SON "HANDLE"
    h_start = plot3(xy0(1), xy0(2), 0, 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
    plot_handles(end+1) = h_start; % Add the handle to our list
    
    for j = 1:length(options)
        option = options(j);
        
        % Appel de la fonction de simulation
        [coup, vbf, t, x, y, z] = Devoir2(option, xy0, vb0, wb0);
        
        % Tracé de la trajectoire en 3D et CAPTURER SON "HANDLE"
        h_traj = plot3(x, y, z, 'Color', colors{j}, 'LineWidth', 2);
        plot_handles(end+1) = h_traj; % Add the handle to our list
    end
    
    % --- Finalisation du graphique ---
    title(['Trajectoire de la balle pour le Coup ' num2str(i)]);
    xlabel('Position X (m)');
    ylabel('Position Y (m)');
    zlabel('Position Z (m)');
    
    % Create the list of text labels in the correct order
    all_legend_entries = ['Point de départ', legend_entries];
    
    % Now, call legend with the specific handles and their corresponding labels
    legend(plot_handles, all_legend_entries, 'Location', 'best');
    
    grid off;       % Active la grille
    axis equal;    % Assure que les échelles sont les mêmes sur tous les axes
    view(3);       % Vue 3D par défaut```

    hold off; % Termine la superposition des tracés
    
end

disp('Les 4 graphiques ont été générés.');
