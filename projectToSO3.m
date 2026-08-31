function Rproj = projectToSO3(R)
% Projects a near-rotation matrix onto SO(3)

    [U, ~, V] = svd(R);
    Rproj = U * V';

    if det(Rproj) < 0
        U(:,3) = -U(:,3);
        Rproj = U * V';
    end
end