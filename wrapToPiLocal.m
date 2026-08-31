function a = wrapToPiLocal(a)
% Wrap angle to [-pi, pi]

    a = mod(a + pi, 2*pi) - pi;
end