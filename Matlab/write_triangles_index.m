function write_trinangles_index(face, filename)

%**************************************************************************
% 
% WRITE_TRIANGLES write Delaunay triangles from  unit sphere points
%    each element represents indexes of keypoints on the unit sphere.
%
% Last modified
%    22 MAR 2015
%
% Author
%    Naoki Chiba
%**************************************************************************

face = face'; 
csvwrite(filename, face);

end
