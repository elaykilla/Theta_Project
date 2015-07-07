function[ points ] = read_sphere_points( filename )

%**************************************************************************
% 
% READ_SPHERE_POINTS reads unit sphere points from the file
%
% Last modified
%    22 MAR 2015
%
% Author
%    Naoki Chiba
%**************************************************************************
M = csvread( filename );
points = transpose( M );

end
