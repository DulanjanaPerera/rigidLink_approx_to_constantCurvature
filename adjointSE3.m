function Ad = adjointSE3(T)
% T: 4x4 homogeneous transform
% Ad: 6x6 adjoint for twists [omega; v]
    R = T(1:3,1:3);
    p = T(1:3,4);
    px = [   0   -p(3)  p(2);
           p(3)   0   -p(1);
          -p(2)  p(1)   0  ];
    Ad = [R, zeros(3,3);
          px*R, R];
end
