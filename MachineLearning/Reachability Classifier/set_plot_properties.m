
function set_plot_properties(varargin)

mode = 'display';

for k = 1:2:length(varargin)
    if strcmpi(varargin{k},'mode')
        mode = varargin{k+1};
    end
end

if strcmpi(mode,'display')
    % Plotting parameters
    hidetitles  = 'y';          % Hide the figure titles?   [y/n]
    hidegrids   = 'n';          % Hide the figure grids?    [y/n]
    hidelegends = 'n';          % Hide the figure legends?  [y/n]
    whitebackg  = 'y';          % Whiten the figure backgrounds? [y/n]
    linewidth   = 1.5;            % Set the default figure linewidth
    markersize  = 6;            % Set the default line markersize
    titlesize   = 14;           % Set the figure title font size (10 pt is the default)
    titlefont   = 'Helvetica';  % Set the figure title fonts
    titleweight = 'bold';       % Set the figure title font weight ( {normal} | bold | light | demi )
    labelsize   = 28;           % Set the figure axes labels font size (10 pt is the default)
    labelfont   = 'Helvetica';  % Set the figure axes labels fonts
    labelweight = 'normal';     % Set the figure axes labels font weight ( {normal} | bold | light | demi )
    windowscale = [0.60 0.80];  % Fraction of [screenwidth, screenheight] to use for figures
    windowcenter= [0.5 0.5];    % Fraction of [screenwidth, screenheight] for figure center position
    
elseif strcmpi(mode,'publication')
    % Plotting parameters
    hidetitles  = 'y';          % Hide the figure titles?   [y/n]
    hidegrids   = 'n';          % Hide the figure grids?    [y/n]
    hidelegends = 'n';          % Hide the figure legends?  [y/n]
    whitebackg  = 'y';          % Whiten the figure backgrounds? [y/n]
    linewidth   = 2;            % Set the default figure linewidth
    markersize  = 14;         	% Set the default line markersize
    titlesize   = 12;           % Set the figure title font size (10 pt is the default)
    titlefont   = 'FixedWidth'; % Set the figure title fonts
    titleweight = 'normal';     % Set the figure title font weight ( {normal} | bold | light | demi )
    labelsize   = 26;           % Set the figure axes labels font size (10 pt is the default)
    labelfont   = 'FixedWidth'; % Set the figure axes labels fonts
    labelweight = 'normal';     % Set the figure axes labels font weight ( {normal} | bold | light | demi )
    windowscale = [0.55 0.75];  % Fraction of [screenwidth, screenheight] to use for figures
    windowcenter= [0.5 0.5];    % Fraction of [screenwidth, screenheight] for figure center position
else
    return          % Do not override anything and escape the function
end

% Override currently-open figure properties (title and label font sizes, gridding, titles, etc), if specified
allaxes = findall(0,'Type','Axes');
allfigs = findall(0,'Type','Figure');
allLines = findall(0,'Type','Line');

set( allaxes, 'FontSize', labelsize);
set( allaxes, 'FontName', labelfont);
set( allaxes, 'FontWeight', labelweight);
if ( length(allaxes) > 1 )
    set( cell2mat(get(allaxes,'Xlabel')), 'FontSize', labelsize);
    set( cell2mat(get(allaxes,'Ylabel')), 'FontSize', labelsize);
    set( cell2mat(get(allaxes,'Zlabel')), 'FontSize', labelsize);
    set( cell2mat(get(allaxes,'Title')),  'FontSize', titlesize);
    set( cell2mat(get(allaxes,'Title')),  'FontName', titlefont);
    set( cell2mat(get(allaxes,'Title')),  'FontWeight', titleweight);
else
    set( get(allaxes,'Xlabel'), 'FontSize', labelsize);
    set( get(allaxes,'Ylabel'), 'FontSize', labelsize);
    set( get(allaxes,'Zlabel'), 'FontSize', labelsize);
    set( get(allaxes,'Title'),  'FontSize', titlesize);
    set( get(allaxes,'Title'),  'FontName', titlefont);
    set( get(allaxes,'Title'),  'FontWeight', titleweight);
end
if strcmpi( hidegrids, 'y' )
    set( findall(allaxes,'XGrid','on'), 'Xgrid', 'off' );
    set( findall(allaxes,'YGrid','on'), 'Ygrid', 'off' );
    set( findall(allaxes,'ZGrid','on'), 'Zgrid', 'off' );
end
if strcmpi( hidetitles, 'y' )
    if ( length(allaxes) > 1 )
        set( cell2mat(get(allaxes,'Title')), 'String', [] );
    else
        set( get(allaxes,'Title'), 'String', [] );
    end
end
if strcmpi( hidelegends, 'y' )
    for k = 1:length(allaxes)
        legend( allaxes(k), 'off' )
    end
end
if strcmpi(whitebackg, 'y')
    set( allfigs, 'Color', [1 1 1] );
end

if ( length(allaxes) > 1 )
    unchanged = ( cell2mat(get(allaxes,'LineWidth')) == get(0, 'DefaultLineLineWidth') );
else
    unchanged = ( get(allaxes,'LineWidth') == get(0, 'DefaultLineLineWidth') );
end
set( allaxes( unchanged ), 'LineWidth', linewidth );

if ( length(allLines) > 1 )
    unchanged = ( cell2mat(get(allLines,'MarkerSize')) == get(0, 'DefaultLineMarkerSize') );
else
    unchanged = ( get(allLines,'MarkerSize') == get(0, 'DefaultLineMarkerSize') );
end
set( allLines( unchanged ), 'MarkerSize', markersize );

scrsz = get(0,'ScreenSize');            % Screensize variable [left, bottom, width, height] (pixels)
set( allfigs, 'OuterPosition', ...
    [(windowcenter(1)-windowscale(1)/2)*scrsz(3), (windowcenter(2)-windowscale(2)/2)*scrsz(4), ...
    windowscale(1)*scrsz(3), windowscale(2)*scrsz(4)] );

end
