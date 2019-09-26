syms x1
syms x2
syms x3
assume(x1,'real')
assume(x2,'real')
assume(x3,'real')
J = [-1.5*sin(x1) - 2.5*sin(x1 + x2) - 2.75*sin(x1 + x2 + x3), -2.5*sin(x1 + x2) - 2.75*sin(x1 + x2 + x3), -2.75*sin(x1 + x2 + x3);
  1.5*cos(x1) + 2.5*cos(x1 + x2) + 2.75*cos(x1 + x2 + x3), 2.5*cos(x1 + x2) + 2.75*cos(x1 + x2 + x3), 2.75*cos(x1 + x2 + x3)];
Jinv = pinv(J);
simplify(Jinv)

% [                                                                  -(300*sin(x1 + 2*x2) - 330*sin(x1 - x3) + 605*sin(x1 + x2 + 2*x3) + 660*sin(x1 + 2*x2 + x3) - 605*sin(x1 + x2) - 330*sin(x1 + x3) - 1026*sin(x1) + 726*sin(x1 + 2*x2 + 2*x3))/(1815*cos(x2 + 2*x3) + 990*cos(2*x2 + x3) + 450*cos(2*x2) + 3025*cos(2*x3) + 1089*cos(2*x2 + 2*x3) - 1815*cos(x2) - 990*cos(x3) - 4564),                                                                   (300*cos(x1 + 2*x2) - 330*cos(x1 - x3) + 605*cos(x1 + x2 + 2*x3) + 660*cos(x1 + 2*x2 + x3) - 605*cos(x1 + x2) - 330*cos(x1 + x3) - 1026*cos(x1) + 726*cos(x1 + 2*x2 + 2*x3))/(1815*cos(x2 + 2*x3) + 990*cos(2*x2 + x3) + 450*cos(2*x2) + 3025*cos(2*x3) + 1089*cos(2*x2 + 2*x3) - 1815*cos(x2) - 990*cos(x3) - 4564)]
% [ (300*sin(x1 + 2*x2) - 180*sin(x1 - x2) - 330*sin(x1 - x3) + 198*sin(x1 + x2 + x3) + 198*sin(x2 - x1 + x3) - 605*sin(x1 + x2 + 2*x3) + 660*sin(x1 + 2*x2 + x3) + 785*sin(x1 + x2) - 330*sin(x1 + x3) - 663*sin(x1) + 363*sin(x1 + 2*x2 + 2*x3))/(1815*cos(x2 + 2*x3) + 990*cos(2*x2 + x3) + 450*cos(2*x2) + 3025*cos(2*x3) + 1089*cos(2*x2 + 2*x3) - 1815*cos(x2) - 990*cos(x3) - 4564), (180*cos(x1 - x2) - 300*cos(x1 + 2*x2) + 330*cos(x1 - x3) - 198*cos(x1 + x2 + x3) + 198*cos(x2 - x1 + x3) + 605*cos(x1 + x2 + 2*x3) - 660*cos(x1 + 2*x2 + x3) - 785*cos(x1 + x2) + 330*cos(x1 + x3) + 663*cos(x1) - 363*cos(x1 + 2*x2 + 2*x3))/(1815*cos(x2 + 2*x3) + 990*cos(2*x2 + x3) + 450*cos(2*x2) + 3025*cos(2*x3) + 1089*cos(2*x2 + 2*x3) - 1815*cos(x2) - 990*cos(x3) - 4564)]
% [             (1298*sin(x1 + x2 + x3) - 660*sin(x1 - x3) - 1100*sin(x1 + x2 - x3) + 198*sin(x2 - x1 + x3) + 1210*sin(x1 + x2 + 2*x3) + 330*sin(x1 + 2*x2 + x3) - 1210*sin(x1 + x2) + 330*sin(x1 + x3) - 363*sin(x1) + 363*sin(x1 + 2*x2 + 2*x3))/(1815*cos(x2 + 2*x3) + 990*cos(2*x2 + x3) + 450*cos(2*x2) + 3025*cos(2*x3) + 1089*cos(2*x2 + 2*x3) - 1815*cos(x2) - 990*cos(x3) - 4564),             (660*cos(x1 - x3) - 1298*cos(x1 + x2 + x3) + 1100*cos(x1 + x2 - x3) + 198*cos(x2 - x1 + x3) - 1210*cos(x1 + x2 + 2*x3) - 330*cos(x1 + 2*x2 + x3) + 1210*cos(x1 + x2) - 330*cos(x1 + x3) + 363*cos(x1) - 363*cos(x1 + 2*x2 + 2*x3))/(1815*cos(x2 + 2*x3) + 990*cos(2*x2 + x3) + 450*cos(2*x2) + 3025*cos(2*x3) + 1089*cos(2*x2 + 2*x3) - 1815*cos(x2) - 990*cos(x3) - 4564)]
