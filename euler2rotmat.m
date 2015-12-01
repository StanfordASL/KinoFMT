
function R = euler2rotmat(ANGLE_VECTOR, SEQUENCE, varargin)

% EULER2ROTMAT Calculate the rotation matrix from a given Euler angle sequence
%    R = EULER2ROTMAT(ANGLE_VECTOR, SEQUENCE) returns the rotation matrix
%    R corresponding to individual orthogonal rotations through angles in 
%    ANGLE_VECTOR according to the rotation sequence SEQUENCE.
%
%    SEQUENCE is a code string indicating about which local axes the
%    sequence of rotations in ANGLE_VECTOR should be made.  SEQUENCE should
%    be specified according to the following code key:
%           1 - rotation about the 1st-axis of the current local frame
%           2 - rotation about the 2nd-axis of the current local frame
%           3 - rotation about the 3rd-axis of the current local frame
%    e.g. '321' denotes a rotation first about the 3rd (z-) axis, then
%    about 2nd (y-) axis of the new frame, and finally about the 1st (x-)
%    axis of the newer frame.  Other common examples for SEQUENCE are '1',
%    '2', '3', '313', '123', etc.  Any number and sequence of rotations can
%    be made, though theoretically 3 is all that is needed to uniquely
%    specify any rotation.
%
%    R = EULER2ROTMAT(ANGLE_VECTOR, SEQUENCE, 'Property1', Value1, 'Property2', Value2, ...)
%    allows you to set the values of the properties of EULER2ROTMAT.  A list
%    and description of tailorable properties can be found below.
%
%    EULER2ROTMAT PROPERTIES
%    -----------------------
%    Angle Type
%    	{'deg'} | 'rad'
%    
%       Specifies the unit type of the rotation angles used in ANGLE_VECTOR
%
%    Variables
%       cell vector of strings
%
%       Specifies custom variable names for the rotation angles used in
%       ANGLE_VECTOR.  
%               Ex: ANGLE_VARIABLES = {'phi', 'theta', 'psi', ...}
%       This allows for the display of custom variables in the symbolic command
%       window output.  If unspecified, default names for angles are 'angle_1',
%       'angle_2', ... 'angle_n'.
%
%    Display
%       {'on'} | 'off'
%
%    The output R should be used in different ways depending on what the 
%    rotations of ANGLE_VECTOR and SEQUENCE correspond to.  R applies
%    differently whether you are rotating a fixed vector or if you are 
%    reorienting your coordinate system.  See the guides below to 
%    understand how you should appropriately use R.
%
%    ------------------------------------
%    Rotate object relative to fixed axes
%    ------------------------------------
%    Returns the rotation matrix that is required to rotate a vector from one
%    orientation to another with a fixed coordinate system.  The angles in
%    ANGLE_VECTOR correspond to the sequence of angles through which the
%    vector would be rotated away from the original fixed coordinate system.
%
%        v(F_1) --> R(ANGLE_VECTOR) --> v'(F_1), frame F_1 remains unchanged
%
%           |                           |
%        v' |     = R(ANGLE_VECTOR) * v |
%           | F_1                       | F_1
%
%    Under this definition, by similarity transformation
%           |                           |
%        M' |     = R(ANGLE_VECTOR) * M |    * R^T(ANGLE_VECTOR)
%           | F_1                       | F_1
%
%    Example: R = EULER2ROTMAT([45, 60, 30], '321') returns R which will,
%    when applied to a vector in an inertial system, yaw it by 45 degrees,
%    pitch it by 60 degrees, then roll it by 30 degrees.  Or, if R is 
%    applied to a coordinate system, R^-1 will represent the orientation
%    matrix that resolves a fixed vector in some inertial coordinate system
%    into new coordinates in the new coordinate system that has been yawed
%    by 45 degrees, pitched by 60 degrees, and rolled by 30 degrees, 
%                   i.e. v_body-fixed = R^-1*v_inertial.
%
%    -------------------------------------------
%    Rotation of an orthogonal coordinate system
%    -------------------------------------------
%    Returns the rotation matrix that is required to rotate a coordinate
%    system from one orientation to another.  The angles in ANGLE_VECTOR
%    correspond to the sequence of angles required to rotate the coordinate
%    system axes one axis at a time into the new orientation.  Because a
%    rotation of each coordinate system axis appears to be a negative rotation
%    of a fixed vector, we have:
%
%        F_1 --> R(ANGLE_VECTOR) --> F_2, vector v remains unchanged [but its representation changes]
%
%          |                            |                             |
%        v |     = R(-ANGLE_VECTOR) * v |     = R^T(ANGLE_VECTOR) * v |    
%          | F_2                        | F_1                         | F_1
%                                      |
%                = O(ANGLE_VECTOR) * v |    , where O = orientation matrix
%                                      | F_1          = R^-1 = R^T
%
%    Under this definition, by similarity transformation
%          |                             |
%        M |     = R^T(ANGLE_VECTOR) * M |    * R(ANGLE_VECTOR)
%          | F_2                         | F_1
%
%    ******* A word of caution: the guides above are consistent with each
%    other, and believed to be correct, but they are unverified.  They
%    should be either both right or they are both wrong.

% ======================================================================= %

% Set defaults
angle_type = 'deg';
angle_variables = cell(1, length(ANGLE_VECTOR));
display = 'on';
for m = 1:1:length(ANGLE_VECTOR)
    angle_variables{m} = ['angle_', num2str(m)];
end

% Check for user settings
for m = 1:2:length(varargin)
    property = varargin{m};
    if strcmpi(property, 'Angle Type')
        angle_type = lower(varargin{m+1});
    elseif strcmpi(property, 'Variables')
        angle_variables = lower(varargin{m+1});
    elseif strcmpi(property, 'Display')
        display = varargin{m+1};
    else
        error('Property not recognized or of invalid format.  Please check your spelling.');
    end
end

% Validate the input for ANGLE_VECTOR and ANGLE_TYPE (if specified), and 
% convert angles in ANGLE_VECTOR to radians if necessary.
if isnumeric(ANGLE_VECTOR) && isreal(ANGLE_VECTOR) && min(size(ANGLE_VECTOR)) == 1
    switch angle_type
        case 'deg'
            ANGLE_VECTOR = (pi/180).*ANGLE_VECTOR;
        case 'rad'
        otherwise
            error(['Invalid input for ANGLE_TYPE.  Please enter as ''deg'' for', ...
                'degrees or ''rad'' for radians.']);
    end
else
    error('Invalid input for ANGLE_VECTOR.  ANGLE_VECTOR must be a numeric vector of all real entries.');
end

% Validate the input for SEQUENCE
if ~isempty(regexp(SEQUENCE, '[^1-3]', 'once'))
    error(['Invalid input for SEQUENCE.  Please make sure SEQUENCE is a string', ...
        ' that includes the digits 1, 2, or 3 only.']);
elseif length(SEQUENCE) ~= length(ANGLE_VECTOR)
    error('Please be sure the number of axes listed in SEQUENCE equals the number of angles in ANGLE_VECTOR');
end

% Validate the input for ANGLE_VARIABLES
if ~iscellstr(angle_variables) || ~isvector(angle_variables)
    error('Invalid input for ANGLE_VARIABLES.  Please specify angle variables as a cell vector of strings');
elseif length(angle_variables) ~= length(ANGLE_VECTOR)
    error('Please be sure the number of strings in ANGLE_VARIABLES equals the number of angles in ANGLE_VECTOR');
end

% Compute the overall rotation matrix corresponding to sum of each
% individual orthogonal rotation in ANGLE_VECTOR as specified by SEQUENCE
R = eye(3);
toolboxes = ver;
if ~isempty(strfind(strcat(toolboxes.Name), 'Symbolic'))        % Check if the symbolic toolbox exists
    R_symbolic = sym(R);
end
s_out = [' '; ...
         '0'; ...
         ' '];

for m = 1:1:length(SEQUENCE)
    angle = ANGLE_VECTOR(m);
    axis = str2double(SEQUENCE(m));
    switch axis
        case 1
            R_intermediate = [1,            0,              0; ...
                              0,            cos(angle),     -sin(angle); ...
                              0,            sin(angle),     cos(angle)];
        case 2
            R_intermediate = [cos(angle),   0,              sin(angle); ...
                              0,            1,              0; ...
                              -sin(angle),  0,              cos(angle)];
        case 3
            R_intermediate = [cos(angle),   -sin(angle),    0; ...
                              sin(angle),   cos(angle),     0; ...
                              0,            0,              1]; 
    end
    R = R_intermediate*R;
    s_out_increment = ['  R_', num2str(axis), '(', angle_variables{m},                       ')    '; ...
                       ' ---', '-',           '-', repmat('-',1,length(angle_variables{m})), '--> ', num2str(m); ...
                       '    ', ' ',           ' ', repmat(' ',1,length(angle_variables{m})), '     '];
    s_out = [s_out, s_out_increment];
    
    if ~isempty(strfind(strcat(toolboxes.Name), 'Symbolic'))
        sym_angle = sym(angle_variables{m});
        switch axis
            case 1
                R_intermediate_symbolic = sym([1, 0, 0; ...
                    0, cos(sym_angle), -sin(sym_angle); ...
                    0, sin(sym_angle), cos(sym_angle)]);
            case 2
                R_intermediate_symbolic = sym([cos(sym_angle), 0, sin(sym_angle); ...
                    0, 1, 0; ...
                    -sin(sym_angle), 0, cos(sym_angle)]);
            case 3
                R_intermediate_symbolic = sym([cos(sym_angle), -sin(sym_angle), 0; ...
                    sin(sym_angle), cos(sym_angle), 0; ...
                    0, 0, 1]);
        end
        R_symbolic = R_intermediate_symbolic*R_symbolic;
    end
end

if strcmpi(display, 'on')
    disp(' ');
    disp('ROTATION SUMMARY');
    disp('****************');
    disp(s_out);

    if ~isempty(strfind(strcat(toolboxes.Name), 'Symbolic'))
        disp(' ');
        disp('Symbolic representation of Rotation Matrix: ');
        pretty(R_symbolic); disp(' ');
    end
end

end