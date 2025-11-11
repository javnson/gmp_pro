% Generate a test data for fuzzy controller.

% This FLC preset is suitable for 2 order underdamped system
[fuzzy_2d_mat, dim1_mesh, dim2_mesh] = get_fuzzy_matrix('underdamped_flc_dsn.fis', 2, [-1.0, 1.0; -1.0, 1.0], [20, 20]);
generate_fuzzy_2d_src('flc_2order_underdamped', 'flc_underdamped', fuzzy_2d_mat, dim1_mesh, dim2_mesh);
