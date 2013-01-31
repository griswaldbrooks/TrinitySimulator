function H = T_2D(x,y,th)
    H = [cos(th), -sin(th), x;
         sin(th),  cos(th), y;
               0,        0, 1];