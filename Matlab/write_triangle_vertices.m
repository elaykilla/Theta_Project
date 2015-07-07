function write_trinangle_vertices(points, face, filename)

%**************************************************************************
% 
% WRITE_TRIANGLE_VERTICES writes Delaunay triangles from  unit sphere points
%    each element represents vertex 3D coordinates of keypoints on the unit sphere.
%
% Last modified
%    22 MAR 2015
%
% Author
%    Naoki Chiba
%**************************************************************************

  [dim_num, face_num] = size( face ); 

  vertex = zeros(9, face_num);

  for j = 1 : face_num
    for n = 1 : 3
        face_id = face(n, j);
        vertex(3*(n-1)+1, j) = points(1, face_id);
        vertex(3*(n-1)+2, j) = points(2, face_id);
        vertex(3*(n-1)+3, j) = points(3, face_id);
    end
  end

   vertex 
   %csvwrite(filename, vertex');
   dlmwrite(filename, vertex', 'precision', '%.6f');
   return
end
