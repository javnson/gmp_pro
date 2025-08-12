% In order to improve the efficience of Fuzzy controller
% we should get the fuzzzy logic LUT (look-up table).
% Then, we may calculate the error and error rate as input.
% Based on Fuzzy Logic LUT we may interpolate linearly 
% to obtain the final output.

%% Function get fuzzy matrix
% input parameter
% fis_file: provide the fis file path and name
% dim input is a number tells the input variables dimention
% range_input is a matrix tells the input variables value range
% mash_input is a matrix tells how to segments these variables

% warning output owns only one demension
% for now, only 2-d input is supported.

% usage example:
% get_fuzzy_matrix('mtr_fuzzy_matrix.fis', 2, [-0.5, 0.5; -0.5, 0.5], [20, 20], ...
%     1, [-0.5, 0.5], [20]);

function [fuzzy_matrix, dim1_mesh, dim2_mesh] = get_fuzzy_matrix(fis_file, ...
    dim_input,   range_input,  mash_input)

fis = readfis(fis_file);

%for()

if dim_input == 2
    dim_max = max(range_input(1,:));
    dim_min = min(range_input(1,:));
    dim_div = (dim_max - dim_min) / mash_input(1);
    dim1_mesh = dim_min : dim_div : dim_max;

    dim_max = max(range_input(2,:));
    dim_min = min(range_input(2,:));
    dim_div = (dim_max - dim_min) / mash_input(2);
    dim2_mesh = dim_min : dim_div : dim_max;

    fuzzy_matrix = zeros(length(dim1_mesh), length(dim2_mesh));

    for i = 1:length(dim1_mesh)
        for j = 1:length(dim2_mesh)
            fuzzy_matrix(i,j) = evalfis(fis,[dim1_mesh(i) dim2_mesh(j)]);

        end
    end

    % Show fuzzy surface
    figure;
    surf(dim1_mesh, dim2_mesh, fuzzy_matrix);

else 
    disp('unsupport demension.')
end


end

