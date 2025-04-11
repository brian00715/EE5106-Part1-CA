function points = generateLetterTrajectories()
    % Scale and offset for the letters
    scale = 0.15;  % Scale factor for letters
    baseX = 0.2;   % Center X position
    baseY = 0;     % Center Y position
    baseZ = 0.6;   % Center Z position
    
    % Generate points for "HELLO"
    points = [];
    
    % Letter spacing
    letterSpacing = 0.05;
    currentX = baseX - 0.2;  % Start position
    
    % H
    h_points = generateH(currentX, baseY, baseZ, scale);
    points = [points; h_points];
    currentX = currentX + letterSpacing;
    
    % E
    e_points = generateE(currentX, baseY, baseZ, scale);
    points = [points; e_points];
    currentX = currentX + letterSpacing;
    
    % L
    l_points = generateL(currentX, baseY, baseZ, scale);
    points = [points; l_points];
    currentX = currentX + letterSpacing;
    
    % L
    l_points = generateL(currentX, baseY, baseZ, scale);
    points = [points; l_points];
    currentX = currentX + letterSpacing;
    
    % O
    o_points = generateO(currentX, baseY, baseZ, scale);
    points = [points; o_points];
end

function points = generateH(x, y, z, scale)
    points = [
        x, y, z;                              % Start bottom left
        x, y, z + scale;                      % Up
        x, y, z + scale/2;                    % Middle
        x + scale/2, y, z + scale/2;          % Middle right
        x + scale/2, y, z;                    % Bottom right
        x + scale/2, y, z + scale;            % Top right
    ];
end

function points = generateE(x, y, z, scale)
    points = [
        x + scale/2, y, z;                    % Start bottom right
        x, y, z;                              % Bottom left
        x, y, z + scale;                      % Top left
        x + scale/2, y, z + scale;            % Top right
        x, y, z + scale;                      % Back to top left
        x, y, z + scale/2;                    % Middle left
        x + scale/2, y, z + scale/2;          % Middle right
    ];
end

function points = generateL(x, y, z, scale)
    points = [
        x, y, z + scale;                      % Start top
        x, y, z;                              % Bottom
        x + scale/2, y, z;                    % Right
    ];
end

function points = generateO(x, y, z, scale)
    t = linspace(0, 2*pi, 20)';              % 20 points for circle
    radius = scale/2;
    points = [
        x + radius * cos(t), ...              % X coordinates
        y + zeros(size(t)), ...               % Y coordinates
        z + radius + radius * sin(t)          % Z coordinates
    ];
end
