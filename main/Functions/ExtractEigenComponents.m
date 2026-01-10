function [largest_eigenval, smallest_eigenval, largest_eigenvec, smallest_eigenvec] = ExtractEigenComponents(eigenval, eigenvec)
%**************************************************************************
% ExtractEigenComponents
%
% Description:
%   Given the eigenvalues and eigenvectors of a matrix, this function 
%   extracts the largest and smallest eigenvalues and their corresponding 
%   eigenvectors.
%
% Inputs:
%   eigenval - square diagonal matrix of eigenvalues (output of eig)
%   eigenvec - matrix whose columns are eigenvectors (output of eig)
%
% Outputs:
%   largest_eigenval   - largest eigenvalue
%   smallest_eigenval  - smallest eigenvalue
%   largest_eigenvec   - eigenvector corresponding to the largest eigenvalue
%   smallest_eigenvec  - eigenvector corresponding to the smallest eigenvalue
%
% Example:
%   [V,D] = eig(cov(randn(100,2)));
%   [l_max, l_min, v_max, v_min] = ExtractEigenComponents(D, V);
%
% Andrea Valmorbida - November 2025
%**************************************************************************

    % Convert eigenvalue matrix to vector
    eigenval_diag = diag(eigenval);

    % Sort eigenvalues in ascending order and get indices
    [eigenval_sorted, evs_idx] = sort(eigenval_diag);

    % Extract smallest and largest eigenvalues
    smallest_eigenval = eigenval_sorted(evs_idx(1));
    largest_eigenval  = eigenval_sorted(evs_idx(end));

    % Extract corresponding eigenvectors
    smallest_eigenvec = eigenvec(:, evs_idx(1));
    largest_eigenvec  = eigenvec(:, evs_idx(end));

end
