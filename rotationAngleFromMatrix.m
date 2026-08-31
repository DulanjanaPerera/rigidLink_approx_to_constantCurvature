function ang = rotationAngleFromMatrix(R)
% Returns geodesic rotation angle from a 3x3 rotation matrix

    c = (trace(R) - 1) / 2;
    c = min(1, max(-1, c));   % numerical safety
    ang = acos(c);
end