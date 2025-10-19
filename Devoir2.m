% FONCTION DU DEVOIR
    function [coup, vbf, t, x, y, z]= Devoir2 (option,xy0,vb0,wb0)

    if norm(vb0) >= 100
        error('La norme de vb0 (%.2f m/s) dépasse la limite de 100 m/s.', norm(vb0));
    end

    C = constantes();
    q0 = [vb0(1); vb0(2); vb0(3); xy0(1) ; xy0(2); 0];
    wb0 = wb0(:);
    q = q0;

    idx = 1
    t_debut = 0;
    t_cur = t_debut;
    t_max = 100;
    t_intervalle = 0.01;

    max_steps = round(t_max / t_intervalle) + 1;
    t = zeros(max_steps, 1);
    x = zeros(max_steps, 1);
    y = zeros(max_steps, 1);
    z = zeros(max_steps, 1);

    % Initialiser la première ligne des vecteurs
    t(idx) = t_cur;
    x(idx) = q(4);
    y(idx) = q(5);
    z(idx) = q(6);

    espilone  =[100; 100; 100; 1e-3; 1e-3; 1e-3];

    params = {C, option, wb0};   


    while t_cur < t_max
        [qs, m, Err] = SEDRK4t0E(q, t_cur,t_cur + t_intervalle, espilone, @derive_q,params);
        q = qs;
    
        t_cur  = t_cur + t_intervalle;
        idx = idx + 1;

        t(idx)  = t_cur;
        x(idx)  = q(4);   
        y(idx)  = q(5); 
        z(idx)  = q(6);  

        if est_dans_coupe(C, q(4),q(5), q(6))
            coup = 0;
            break
        elseif est_dans_region_boisee(C, q(4), q(5))
            coup = 2;
            break
        elseif q(6) <= 0
            coup = 1;
            break;
        end


        if t_cur >= t_max
            coup = 1; 
        end


    end
     vbf = q(1:3);
     t = t(1:idx);
    x = x(1:idx);
    y = y(1:idx);
    z = z(1:idx);

end



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

function Fg = force_gravite(C) %Prends la masse
    Fg = C.M_B * C.G_VECTEUR; % [N]
end

function Fv = force_visqueuse(C, V)
    % Frottement visqueux : Fv = - 1/2 * rho * C_V * A * |v| * v
    vnorm = norm(V);
    Fv = -(1/2) * C.RHO * C.C_V * C.A * vnorm * V; % [N]
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
    Fg = force_gravite(C);
    Fv = force_visqueuse(C,V);
    a = (Fg + Fv) / C.M_B;
end

function a = acc_gravite_visqueux_magnus(C,V,WB)
    Fg = force_gravite(C);
    Fv = force_visqueuse(C,V);
    Fm = force_magnus(C ,V, WB);
    a = (Fg + Fv + Fm) / C.M_B;
end


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

function q_derive = derive_q(q0, ~, params)
    C      = params{1};
    option = params{2};
    wb0    = params{3};
    V = q0(1:3);                        
    a = accelerationSelonOption(C, option, V, wb0); 
    q_derive = [a(1); a(2); a(3); q0(1); q0(2); q0(3)];
end


function [conv, Err]=ErrSol(qs1,qs0,epsilon)
% Verification si solution convergee
%   conv      : variable logique pour convergence
%               Err<epsilon pour chaque elements
%   Err       : Difference entre qs1 et qs0
%   qs1       : nouvelle solution
%   qs0       : ancienne solution
%   epsilon   : précision pour chaque variable
nbele=length(qs0);
Err=(qs1-qs0);
conv=1;
for i=1:nbele
  conv=conv & abs(Err(i)) < epsilon(i);
end
end


% Solution equations differentielles par methode de RK4
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


function [qs, m, Err]=SEDRK4t0E(q0,t0,tf,epsilon,derive_q, params)
    %   qs        : solution [q(tf)]
    %   q0        : conditions initiales [q(t0)]
    %   t0        : temps initial
    %   tf        : temps final
    %   epsilon   : précision pour chaque variable
    %   g         : membre de droite de ED.
    m=1;nbi=1;DeltaT=(tf-t0);
    % Solution avec m=1
    qs1=SEDRK4t0(q0,t0,DeltaT,derive_q,params);
    [conv, Err]=ErrSol(qs1,q0,epsilon);
    qs2=qs1;
    % Iteration avec m>1
    while ~conv
      DeltaT=DeltaT/2;
      m=m+1;nbi=nbi*2;
      qs2=q0;t2=t0;
      for i=1:nbi
        qs2=SEDRK4t0(qs2,t2,DeltaT,derive_q,params);
        t2=t2+DeltaT;
      end
      [conv, Err]=ErrSol(qs2,qs1,epsilon);
      qs1=qs2;
      if m>10
        break;
      end
    end
    qs=qs2;
end

