
function C = cuboid(a, b, varargin)
    % CUBOID   Computes the vertices of cuboid
    %
    %    Finds the vertices of the faces of a cuboid given its length a
    %    along the x-axis, its length b along the y-axis, and its length c
    %    along the z-axis, with centroid at the origin.
    %
    %    C = CUBOID(a,b) returns the vertices of an axes-aligned rectangle
    %    to cell array C, assuming a 2D case.
    % 
    %    C = CUBOID(a,b,'showplot') creates a graphical representation of
    %    the rectangle generated.  The default, 'hideplot', hides the plot.
    %
    %    C = CUBOID(a,b,'showplot', v, R) translates the centroid of the 
    %    rectangle by vector v and rotates it by rotation matrix R.  v must
    %    be a numeric vector of length 2 and R must be a 2x2 numeric matrix.
    %
    %    C = CUBOID(a,b,c) returns the vertices of the axes-aligned cuboid
    %    to cell array C, with each matrix entry of C listing the vertex
    %    coordinates of a particular face as columns, in CCW order about the
    %    positive face normal
    %
    %    C = CUBOID(a,b,c,'showplot', v, R) translates the centroid of the 
    %    cuboid by vector v and rotates it by rotation matrix R.  v must be
    %    a numeric vector of length 3 and R must be a 3x3 numeric matrix.
    %
    
    is_2D       = false;
    v           = [0,0,0]';
    R           = eye(3);
    tol         = 1e-8;
    plotoption  = 'hideplot';
    
    try 
        if isempty(varargin) || ischar(varargin{1})
            % 2D case (the variable 'c' was not defined)
            is_2D   = true;
            v       = [0,0]';
            R       = eye(2);
            
            plotoption  = varargin{1};
            v           = varargin{2};
            R           = varargin{3};
            
        elseif isnumeric(varargin{1})
            c           = varargin{1};
            plotoption  = varargin{2};
            v           = varargin{3};
            R           = varargin{4};
        end
    catch ME
    end

    if isnumeric(v) && isvector(v) && (( ~is_2D && length(v) == 3 ) || ( is_2D && length(v) == 2 ))
        if size(v,2) ~= 1
            v = v';
        end
    else
        error('The translation vector v must be a numeric vector of appropriate dimension.');
    end

    if isnumeric(R) && size(R,1) == size(R,2) && (( ~is_2D && max(size(R)) == 3 ) || ( is_2D && max(size(R)) == 2 ))
        if det(R) - 1 > tol
            error(['The matrix R does not appear to be a valid rotation matrix (det(R) = ', num2str(det(R)),' is not within tolerance of +1']);
        end
    else
        error('The rotation matrix R must be a square, numeric matrix of appropriate size.');
    end
    
    if is_2D
        C  = 0.5*[ a    a   -a   -a; ...
                   b   -b   -b    b ];
        n_faces = 1;
        for k = 1:(n_faces*4)
            C(:,k) = R*C(:,k);
        end
        C = { C + repmat(v, 1, 4) };
        
    else
        F1 = 0.5*[ a    a    a    a; ... 
                   b   -b   -b    b; ...
                   c    c   -c   -c ];
        F2 = 0.5*[-a   -a   -a   -a; ...
                   b    b   -b   -b; ...
                   c   -c   -c    c ];
        F3 = 0.5*[ a   -a   -a    a; ...
                  -b   -b   -b   -b; ...
                   c    c   -c   -c ];
        F4 = 0.5*[ a   -a   -a    a; ...
                   b    b   -b   -b; ...
                   c    c    c    c ];   
        F5 = 0.5*[ a    a   -a   -a; ...
                   b    b    b    b; ...
                   c   -c   -c    c ];  
        F6 = 0.5*[ a    a   -a   -a; ...
                   b   -b   -b    b; ...
                  -c   -c   -c   -c ];

        n_faces = 6;
        C = [F1 F2 F3 F4 F5 F6];
        for k = 1:(n_faces*4)
            C(:,k) = R*C(:,k);
        end
        C = mat2cell( C + repmat(v, 1, n_faces*4), 3, 4.*ones(1,n_faces) );
    end
    
    if strcmpi(plotoption, 'showplot') == 1
        figure; hold on;
        patch_color = [0.1 0.1 0.8];
        if is_2D
            plot( C{1}(1,:), C{1}(2,:), 'ok' );
            patch( C{1}(1,:), C{1}(2,:), patch_color );
            plot3( v(1), v(2), 'ok', 'MarkerFaceColor', 'k' );
            title_string = ['Rectangle of Sides a = ', num2str(a), ', b = ', num2str(b), ...
                ', Centered at v = [', num2str(v'), ']^T'];
        else
            for k = 1:n_faces
                plot3( C{k}(1,:), C{k}(2,:), C{k}(3,:), 'ok' );
                patch( C{k}(1,:), C{k}(2,:), C{k}(3,:), patch_color );
            end
            plot3( v(1), v(2), v(3), 'ok', 'MarkerFaceColor', 'k' );
            title_string = ['Cuboid of Sides a = ', num2str(a), ', b = ', num2str(b), ...
                ', c = ', num2str(c), ', Centered at v = [', num2str(v'), ']^T'];
            zlabel('z');
        end
        alpha(0.5); grid on; axis equal;
        title(title_string); xlabel('x'); ylabel('y');
    end
end
