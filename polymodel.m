
function [vertices, faces, halfedges, N, C, f] = polymodel(C, varargin)
% POLYMODEL   Generates a geometric polyhedral model from 3D coordinates
%
% doubly connected edge list data structure
%
% give vertex coordinates about each face, in CCW order about the outward
% normal vector to each face
%
% every vertex coordinate must be repeated at least 3 times for it to be a
% valid (closed) 3D polygon
%
% specify coordinates in columns, i.e. x-coords along row 1, y along row 2,
% etc...
%
%    for k = 1:length(N)
%         nodes = sum(N(1:k))-N(1) + (1:1:N(k));
%         patch(C(1,nodes), C(2,nodes), C(3,nodes), [0.1 0.1 0.8]);
%    end
%
% VARARGIN
%    'showplot' shows the plot (default is to hide)
%    'hidedisp' hides the command window display (default is to show)

plotoption = 'hideplot';
dispoption = 'showdisp';

try
    plotoption = varargin{1};
    dispoption = varargin{2};
catch ME
end

% Error checking
if isvector(C) && iscell(C)
    if size(C,1) ~= 1
        C = C';
    end
else
    error('C must be a cell vector of numeric matrices.');
end

D = zeros(size(C));
N = zeros(size(C));
for p = 1:length(C)
    D(p) = size(C{p},1);
    N(p) = size(C{p},2);
    if ~isnumeric(C{p})
        error('C must be a cell vector of numeric matrices.');
    end
end
if D(p) ~= D(1)
    error('All vertex coordinate matrices must have the same number of rows.');
end
n_faces = length(C);
C = cell2mat(C);

% Plot the entered coordinates and faces
if D(1) <= 3 && strcmpi(plotoption, 'showplot')
    figure; hold on;
    if D(1) == 2
        plot( C(1,:), C(2,:), 'ok' );
    elseif D(1) == 3
        plot3( C(1,:), C(2,:), C(3,:), 'ok' );
        for k = 1:n_faces
            nodes = sum(N(1:k))-N(1) + (1:1:N(k));
            patch(C(1,nodes), C(2,nodes), C(3,nodes), [0.1 0.1 0.8]);
        end
        zlabel('z');
    end
    alpha(0.5); grid on; axis equal;
    title('Graphical Representation of the Input Polyhedral Model');
    xlabel('x'); ylabel('y');
end

% Initialize the faces, vertices, and halfedges data structures
faces = struct('halfedge', []);
vertices = struct('coords', [], 'halfedge', []);
halfedges = struct('origvertex', [], 'twin', [], 'face', [], 'previous', [], 'next', []);
f = zeros(4, n_faces);

% Create a from-to coordinate pairs matrix for identifying twin half-edges
fromto_pairs = zeros(2*size(C,1),size(C,2));
for k = 1:n_faces
    nodes = sum(N(1:k))-N(1) + (1:1:N(k));
    fromto_pairs(1:3, nodes) = C(:,nodes);
    fromto_pairs(4:6, nodes) = C(:,circshift(nodes,[0 -1]));
end

% Calculate the vertices, halfedges, faces, and primitives associated with
% the polyhedral model
halfedge0 = 1;
face = 1;
for p = 1:n_faces
    n = N(p);
    V = C( :, sum(N(1:p))-N(1) + (1:1:n) );
    
    faces.halfedge( face ) = halfedge0;
    
    for q = 1:1:n
        halfedge = halfedge0 + (q-1);
        v = V( :, q );
        if isempty(vertices.coords) || ~ismember( v', vertices.coords', 'rows')
            vertex = size( vertices.coords, 2 ) + 1;
            vertices.coords( :, vertex ) = v;
            vertices.halfedge( vertex ) = halfedge0 + (q-1);
        else
            vertex = find( ismember( vertices.coords', v', 'rows') == 1 );
        end
        halfedges.origvertex( halfedge ) = vertex;
        halfedges.twin( halfedge )      = find( ismember( fromto_pairs', [V(:,mod(q,n)+1); V(:,q)]', 'rows' ) );
        halfedges.face( halfedge )      = face;
        halfedges.previous( halfedge )  = halfedge0 + mod( q-2, n );
        halfedges.next( halfedge )      = halfedge0 + mod( q, n );
    end
    
    % Check if three vertices v1, v2, v3 of the current face are collinear;
    % if not, fit a plane f = ax + by + cz + d = 0 to the vertices, with 
    % the normal vector [a b c] = u12 x u23 pointing outward towards f > 0
    collinear = 1; q = 1;
    while collinear == 1 && q <= n
        v1 = V( :, q );
        v2 = V( :, mod(q,n)+1 );
        v3 = V( :, mod(q+1,n)+1 );
        u12 = (v2 - v1)./norm(v2-v1,2);
        u23 = (v3 - v2)./norm(v3-v2,2);
        if abs( dot( u12, u23 ) ) ~= 1
            collinear = 0;
            f(:,p) = [cross( u12, u23 ); -dot( cross(u12,u23),v1 )];
        end
    end
    
    halfedge0 = halfedge0 + n;
    face = face + 1;
end
n_vertices = size(vertices.coords, 2);
n_edges = size(halfedges.origvertex, 2)/2;

% Display the results
if strcmpi(dispoption, 'showdisp')
    disp('MODEL PROPERTIES');
    disp('----------------');
    disp(['Number of unique edges   : ', num2str(n_edges)]);
    disp(['Number of unique faces   : ', num2str(n_faces)]);
    disp(['Number of unique vertices: ', num2str(n_vertices)]);
end

end
