
function savefigs(varargin)

% SAVEFIGS Save open figures
%    SAVEFIGS() saves all figures in JPEG format and compresses them to
%    'Saved Figures.zip'.  Works by calling SAVEAS repeatedly for each figure.
%
%    SAVEFIGS('Property1', Value1, 'Property2', Value2, ...)
%    allows you to set the values of the properties of SAVEFIGS.  A list
%    and description of tailorable properties can be found below.
%
%    SAVEFIGS PROPERTIES
%    -------------------
%    Format
%       string | cell vector of strings
%
%       Denotes the format in which to save figure files.  Enter a single
%       string to save each figure in that format, or else enter a cell
%       vector of individual format designations. JPEG ('jpg') is the default.
%       
%       Valid options:
%           'bmp', 'emf', 'eps', 'epsc', 'fig', 'hdf'
%           'jpg', 'jpeg', 'pdf', 'png', 'tif', 'tiff'
%
%    Save
%       numeric vector | {'all'} | 'visible' | 'hidden'
%
%       Specifies which figures to save.  Enter a numeric vector of
%       figure handles to save those figures specifically, or enter Save as
%       'all' to save all figures, 'visible' to save just those that are
%       visible, or 'hidden' to save just those that are hidden.
%
%    SaveAs
%       cell vector of strings | {'Default'}, 'Title', 'Header'
%
%       Specifies the filenames for each saved figure.  List filenames
%       individually as a cell vector, or:
%           - Choose 'Default' to save in default naming format (e.g. "Figure #")
%           - Choose 'Title' to save by plot title
%           - Choose 'Header' to save by figure window header (i.e. figure name)
%
%    SaveDir
%       string | {pwd}
%
%       Defines the save directory for figure files and/or zipped figures.
%       SaveDir can be a relative name (indicating a subfolder of the
%       current directory) or an absolute name (full path).  If SaveDir
%       does not already exist, a new folder is created.
%
%    ZipMode
%       'GNU zip', 'tar', {'zip'}, 'none'
%
%       Chooses the compression method used to zip figure files.
%
%    ZipName
%       string | {'Saved Figures'} | []
%
%       Sets the zip filename used to store figure files.  ZipName must be 
%       a relative name.  Enter as [] to prevent any zipping.
%
%    ----------------------------------------------------------------------
%
%    *** Keep in mind any figure filenames that contain '/', '.' or any 
%    other invalid filename characters cannot be saved using those
%    characters.  SAVEFIGS automatically replaces any instances of 
%    unapproved characters with an underscore: '_'.

% Set defaults
savedir         = pwd;            	% SAVEDIR
figures         = 'All';           	% SAVEAS
filenames       = 'Default';        % FILENAMES
formats         = 'jpg';           	% FORMATS
zip_filename    = 'Saved Figures'; 	% ZIPNAME
zip_method      = 'zip';          	% ZIPMODE
tol             = 1e-20;

% Extension vs. Format
format_v_ext = {'bmp', '.bmp'; ...
                'emf', '.emf'; ...
                'eps', '.eps'; ...
                'epsc', '.eps'; ...
                'fig', '.fig'; ...
                'hdf', '.hdf'; ...
                'jpg', '.jpg'; ...
                'jpeg', '.jpg'; ...
                'pdf', '.pdf'; ...
                'png', '.png'; ...
                'tif', '.tif'; ...
                'tiff', '.tif'};

% Determine user settings
for K = 1:2:length(varargin)   
    property = varargin{K};
    selection = varargin{K+1};
    
    if strcmpi(property, 'savedir')
        if ischar(selection)
            savedir = selection;
        else
            error('Invalid entry for the ''SaveDir'' property.  Please enter a valid string for the save directory of the figure files.');
        end
    elseif strcmpi(property, 'save')
        if ischar(selection) || (isnumeric(selection) && isvector(selection) && sum(mod(selection,1)) < length(selection)*tol)
            figures = selection;
        else
            error(['Invalid entry for the ''Save'' property.  Please enter ''All'' to save all figures, ''Visible'' to ', ...
                'save just the open figures, ''Hidden'' to save just the hidden figures, or enter a numeric vector of figure numbers to-be-saved.']);
        end
    elseif strcmpi(property, 'saveas')
        if iscell(selection) || ischar(selection)
            filenames = selection;
        else
            error(['Invalid entry for the ''filenames'' property.  Please enter ''Titles'' to save figures as their plot ', ...
                'titles, ''Headers'' to save as their window header titles, or enter a cell array of strings to create custom filenames.']);
        end
    elseif strcmpi(property, 'format')
        if iscell(selection) || ischar(selection)
            formats = selection;
        else
            error(['Invalid entry for the ''Format'' property.  Please enter a single valid image format string, or a ', ...
                'cell array of format strings for each figure to-be-saved.']);
        end
    elseif strcmpi(property, 'zipname')
        if ischar(selection)
            zip_filename = selection;
        else
            error('Invalid entry for the ''ZipName'' property.  Please enter a string for the zip filename, or ''[]'' to prevent zipping.');
        end
    elseif strcmpi(property, 'zipmode')
        if ischar(selection)
            zip_method = selection;
        else
            error('Invalid entry for the ''zipmode'' property.  Please enter a valid string for the zip compression method to be used.');
        end
    else
        error(['Unknown property "', property, '".  Please check your spelling.']);
    end
end

% Determine the handles of the figures to-be-saved
if ischar(figures)
    if strcmpi(figures, 'All')
        set(0, 'ShowHiddenHandles', 'on');
        fighandles = sort(get(0, 'Children'));
        
    elseif strcmpi(figures, 'Visible')
        set(0, 'ShowHiddenHandles', 'off');
        fighandles = sort(get(0, 'Children'));
        
    elseif strcmpi(figures, 'Hidden')
        set(0, 'ShowHiddenHandles', 'off');
        visiblehandles = get(0, 'Children');
        set(0, 'ShowHiddenHandles', 'on');
        allhandles = get(0, 'Children');
        fighandles = sort(setdiff(allhandles, visiblehandles));
    end
else
    set(0, 'ShowHiddenHandles', 'on');
    allhandles = get(0, 'Children');
    
    if sum(ismember(figures, allhandles)) == length(figures)
        fighandles = figures;
    else
        error('Improper entries for ''Save''.  At least one of the figure numbers specified are not available for saving (i.e. does not exist).');
    end
end
N = length(fighandles);

% Check to see if any figures exist
if N == 0
    error('Cannot use ''savefigs''.  No figures are currently open.  Please open in MATLAB the figures you wish to save before running ''savefigs''.');
end

% Determine the desired filenames for the figures to-be-saved
if ischar(filenames)
    figfiles = cell(1,N);
    
    if strcmpi(filenames, 'Title')
        for K = 1:N
            figaxes = get(fighandles(K), 'CurrentAxes');
            figfiles{K} = get(figaxes, 'Title');
            
            % If any titles/headers were empty, replace with a default name
            if isnumeric(figfiles{K})
                figfiles{K} = ['Figure ', num2str(fighandles(K), '%-1d')];
                disp(['Figure ', num2str(K), ' could not be saved by ''Title'' (figure title is not defined).  Using the default name instead...']);
            end
            
        end
    elseif strcmpi(filenames, 'Header')
        for K = 1:N
            figfiles{K} = get(fighandles(K), 'Name');
            
            % If any titles/headers were empty, replace with a default name
            if isnumeric(figfiles{K})
                figfiles{K} = ['Figure ', num2str(fighandles(K), '%-1d')];
                disp(['Figure ', num2str(K), ' could not be saved by ''Header'' (window header is not defined).  Using the default name instead...']);
            end
            
        end
    elseif strcmpi(filenames, 'Default')
        figfiles = cellstr([repmat('Figure ',N,1), num2str(fighandles, '%-1d')]);
        
    end
else
    if length(filenames) ~= N
        error('Invalid entry for ''filenames''.  The number of filenames must equal the number of figures to-be-saved.');
    end
    figfiles = filenames;
end

% Replace any unapproved filename characters with underscores
figfiles = regexprep(figfiles, '[\/*.?|:]', '_');

% Determine the figure formats for the figures to-be-saved
if ischar(formats)
    figformats = cellstr(repmat(formats,N,1));
else
    if length(formats) ~= N
        error(['Improper entry for ''Formats''.  When specifying custom formats for each figure in a cell array of strings, ' ...
            'the number of strings must equal the number of figures to-be-saved.']);
    end
    figformats = formats;
    
    figformats = cell(N*length(formats));
    for K = 1:length(formats)
        figformats(((K-1)*N+1):K*N) = cellstr(repmat(formats{K},N,1));
    end
end 

% Make new figure folder if necessary
if ~exist(savedir, 'dir')
    if isempty(regexp(savedir, ['\', filesep], 'once'));
        savedir = [pwd, filesep, savedir];
    end
    mkdir(savedir);
end

% Save figures
fullfigfilenames = cell(N,1);
extensions = cell(N,1);
wb = waitbar(0, 'Saving figures...');
for K = 1:N
    fullfigfilenames{K} = [savedir, filesep, figfiles{K}];
    saveas(fighandles(K), fullfigfilenames{K}, figformats{K});
    extensions{K} = format_v_ext{ strcmp(figformats{K}, format_v_ext(:,1)), 2 };
    waitbar(K/N, wb, ['Figure ', num2str(fighandles(K),'%1d'), ' has been saved (', num2str(K,'%1d'), ' out of ', num2str(N,'%1d'), ' complete)']);
end
close(wb);

disp(' ');
disp('Figures saved...');

% Determine full zip filenames, then zip the figure files
%fullfigfilenames = cellstr(strjust([strjust(char(fullfigfilenames),'right'), char(extensions)], 'left'));
%fullzip_filename = [savedir, filesep, zip_filename];
figfilenames    = cellstr(strjust([strjust(char(figfiles),'right'), char(extensions)], 'left'));

if ~isempty(zip_filename) && ~strcmpi(zip_method, 'none')
    home_directory = pwd;
    cd(savedir);
    if strcmpi(zip_method, 'zip');
        zip(zip_filename, figfilenames);
    elseif strcmpi(zip_method, 'GNU zip') || strcmpi(zip_method, 'gzip');
        gzip(figfilenames);
    elseif strcmpi(zip_method, 'tar');
        tar(zip_filename, figfilenames);
    else
        error('Unable to zip the figure files.  Invalid entry for the ''zipmode'' property.  Please enter a valid zip compression method.');
    end
    
    % Delete figure files outside the zip drive
    for K = 1:N
        delete(figfilenames{K})
    end
    
    cd(home_directory)
    disp('Figures zipped...');
end

disp('Done...');
disp(' ');

end
