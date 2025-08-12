% Generate a test data for fuzzy controller.

[fuzzy_2d_mat, dim1_mesh, dim2_mesh] = get_fuzzy_matrix('mtr_fuzzy_dsn.fis', 2, [-1.0, 1.0; -1.0, 1.0], [20, 20]);

generate_fuzzy_2d_src('fuzzy_2d', 'fuzzy_matrix', fuzzy_2d_mat, dim1_mesh, dim2_mesh);
