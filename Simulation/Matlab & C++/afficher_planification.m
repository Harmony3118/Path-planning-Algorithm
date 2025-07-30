function afficher_planification(map, path, explored, start, goal)
% AFFICHER_PLANIFICATION - Affiche la carte, les points explorés, le chemin et les orientations

    figure;
    show(map);         % Affiche la carte occupancyMap
    hold on;

    % === 1. Affichage des états explorés avec orientation ===
    % if ~isempty(explored)
    %     quiver(explored(:,1), explored(:,2), ...
    %            cos(explored(:,3)), sin(explored(:,3)), ...
    %            0.3, 'Color', [0.3 0.6 1], 'MaxHeadSize', 2);
    % end

    % === 1. Affichage des états explorés reliés à leur parent ===
    if ~isempty(explored)
        for i = 1:size(explored,1)
            % Trace une ligne de chaque parent à chaque enfant
            plot([explored(i,1), explored(i,4)], [explored(i,2), explored(i,5)], ...
                 'Color', [0.3 0.6 1], 'LineWidth', 1);
        end
        % Optionnel : afficher les orientations des enfants
        % quiver(explored(:,1), explored(:,2), ...
        %        cos(explored(:,3)), sin(explored(:,3)), ...
        %        0.3, 'Color', [0.3 0.6 1], 'MaxHeadSize', 2);
    end

    % === 2. Affichage du chemin suivi ===
    if ~isempty(path)
        plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2); % ligne rouge
        % Indique les changements de direction
        for i = 2:size(path,1)
            if abs(angdiff(path(i,3), path(i-1,3))) > deg2rad(1)
                plot(path(i-1,1), path(i-1,2), 'o', 'MarkerSize', 5, 'LineWidth', 1.5);
            end
        end
        % Ajoute les flèches d'orientation du chemin
        quiver(path(:,1), path(:,2), ...
               cos(path(:,3)), sin(path(:,3)), ...
               0.4, 'r', 'MaxHeadSize', 3);
    end

    % === 3. Départ et arrivée ===
    plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot(goal(1), goal(2), 'mo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);

    % === 4. Légende ===
    legend({'Exploration', 'Chemin', 'Virage', 'Départ', 'Arrivée'}, ...
           'Location', 'bestoutside');
    title('Planification avec orientation');
    xlabel('X'); ylabel('Y');
end

function d = angdiff(a, b)
% ANGLDIFF - Différence angulaire minimale entre deux angles
    d = mod(a - b + pi, 2*pi) - pi;
end
