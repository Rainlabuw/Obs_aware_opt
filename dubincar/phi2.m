function [y,k] = phi2(x)   
    p = [1;0];

    %p = [2;2];


    k = 1*0.1*((x-p)'*(x-p) + 0.1);

    % k = 1*0.1*((x-p)'*(x-p) + 0.1);


    % r = 2*(0.5-rand(size(x)));
    r = (0.5-rand(size(x)));
    
    
    % if norm(r)>= k
    %     r = k*r/norm(r);
    % end
    
    r = k*r/norm(r);


    y = x + r;

    % y = x ;


end