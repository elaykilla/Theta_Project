function sphere_delaunay_file01 ( )

%*****************************************************************************80
%
%% SPHERE_DELAUNAY_MAIN tests TRIANGULATION_NEIGHBOR_TRIANGLES.
%
%  Licensing:
%
%    This code is distributed under the GNU LGPL license. 
%
%  Modified:
%
%    03 May 2010 and 22 March 2015
%
%  Author:
%
%    John Burkardt and Naoki Chiba
%
  fprintf ( 1, '\n' );
  fprintf ( 1, 'SPHERE_DELAUNAY_MAIN\n' );
  fprintf ( 1, '  Read keypoints on the unit sphere.\n' );
  fprintf ( 1, '  Call SPHERE_DELAUNAY to compute the Delaunay triangulation.\n' );
  fprintf ( 1, '  Call TRIANGULATION_NEIGHBOR_TRIANGLES to get triangle neighbors.\n' );
  fprintf ( 1, '  Call STRI_VERTICES_TO_AREAS to get triangle areas.\n' );
%
%  Read points on the unit sphere from a text file.
%
  filename = './data/Zenkoji5_6.txt';
  filename_triangles = './data/trianglesPoint3D_Zenkoji5_6.txt';
 
  [ xyz ] = read_sphere_points ( filename  );
  [m, n] = size(xyz);
  r8mat_transpose_print ( 3, n, xyz, '  Data points:' );
%
%  Compute the Delaunay triangulation.
%
  [ face_num, face ] = sphere_delaunay ( n, xyz );
%
%  Verify Euler's formula on a sphere.
%
  fprintf ( 1, '\n' );
  fprintf ( 1, '  Check Euler''s formula on a sphere:\n' );
  fprintf ( 1, '\n' );
  fprintf ( 1, '  Faces    = %d\n', face_num );
  fprintf ( 1, '  Vertices = %d\n', n );
  fprintf ( 1, '  Edges    = %d\n', ( 3 * face_num ) / 2 );
  fprintf ( 1, '\n' );
  fprintf ( 1, '  F+V-E-2  = %d\n', face_num + n - ( 3 * face_num ) / 2 - 2 );
%
%  Print the triangulation.
%
  i4mat_transpose_print ( 3, face_num, face, '  Delaunay triangles' );
%
%  Compute the triangle neighbor array.
%
  face_neighbors = triangulation_neighbor_triangles ( 3, face_num, face );
%
%  Print neighbor array.
%
  i4mat_transpose_print ( 3, face_num, face_neighbors, '  Triangle neighbors' );
%
%  Compute the areas.
%
  r = 1;
  area = zeros ( face_num, 1 );
  for i = 1 : face_num
    i1 = face(1,i);
    i2 = face(2,i);
    i3 = face(3,i);
    area(i) = stri_vertices_to_area ( r, xyz(1:3,i1), xyz(1:3,i2), ...
      xyz(1:3,i3) );
  end
  r8vec_print ( face_num, area, '  Spherical area of triangles' );

  area_sum = sum ( area(1:face_num) );
  fprintf ( 1, '\n' );
  fprintf ( 1, '  Area sum = %f\n', area_sum );
  fprintf ( 1, '  4 * PI =   %f\n', 4 * pi );

  write_triangle_vertices(xyz, face, filename_triangles);

  return
end
