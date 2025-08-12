%% @brief Generate a Fuzzy Logic Look-Up Table (LUT) from a .fis file.
%
% @details This function aims to improve the runtime efficiency of a fuzzy
% controller. It pre-calculates the outputs of a fuzzy system for all
% discrete input points, generating a look-up table (control surface).
% In a real-time control loop, this allows for rapid calculation of the
% control output via linear interpolation, avoiding complex real-time
% fuzzy inference.
%
% @param fis_file (string) The path and name of the .fis file.
% @param dim_input (integer) The number of input variables (dimensions).
% @param range_input (matrix) An Nx2 matrix specifying the value range for
%                        each input variable. Each row corresponds to an
%                        input, e.g., `[-1, 1]`.
% @param mash_input (vector) A vector with N elements, specifying the number
%                        of segments for each input range.
%
% @retval fuzzy_matrix The generated Look-Up Table (LUT), a 2D matrix.
% @retval dim1_mesh The meshgrid vector for the first input dimension.
% @retval dim2_mesh The meshgrid vector for the second input dimension.
%
% @note This version only supports fuzzy systems with 2 inputs and 1 output.
%
% @example
%   % Define input range as [-1, 1] and divide each dimension into 50 segments.
%   range = [-1, 1; -1, 1];
%   mash = [50, 50];
%   [matrix, m1, m2] = get_fuzzy_matrix('mtr_fuzzy_dsn.fis', 2, range, mash);
%
function [fuzzy_matrix, dim1_mesh, dim2_mesh] = get_fuzzy_matrix(fis_file, ...
    dim_input,   range_input,  mash_input)

% Read the .fis file
fis = readfis(fis_file);

% Only process the 2D input case
if dim_input == 2
    % --- Create meshgrid for the first dimension ---
    dim_max = max(range_input(1,:));
    dim_min = min(range_input(1,:));
    % Calculate the step size. Note: mash_input is the number of segments,
    % so it will generate (mash_input + 1) points.
    dim_div = (dim_max - dim_min) / mash_input(1);
    dim1_mesh = dim_min : dim_div : dim_max;

    % --- Create meshgrid for the second dimension ---
    dim_max = max(range_input(2,:));
    dim_min = min(range_input(2,:));
    dim_div = (dim_max - dim_min) / mash_input(2);
    dim2_mesh = dim_min : dim_div : dim_max;

    % Initialize the output matrix
    fuzzy_matrix = zeros(length(dim1_mesh), length(dim2_mesh));

    % --- Iterate through all grid points to calculate the fuzzy output ---
    for i = 1:length(dim1_mesh)
        for j = 1:length(dim2_mesh)
            % Use evalfis to calculate the crisp output at the grid point (i,j)
            fuzzy_matrix(i,j) = evalfis(fis,[dim1_mesh(i) dim2_mesh(j)]);
        end
    end

    % --- Visualize the control surface ---
    figure;
    surf(dim1_mesh, dim2_mesh, fuzzy_matrix);
    title('Fuzzy Logic Control Surface');
    xlabel(fis.Inputs(1).Name);
    ylabel(fis.Inputs(2).Name);
    zlabel(fis.Outputs(1).Name);
    colorbar;

else
    % For unsupported dimensions, display a message
    disp('Unsupported input dimension. Only 2D input is currently supported.')
end


end
