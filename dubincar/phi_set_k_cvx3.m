function k = phi_set_k_cvx3(x)   
    p = [1;0];

    %p = [2;2];


    k = 0.1*(norm(x-p)+ 0.1);
    % r = 2*(0.5-rand(size(x)));
    r = (0.5-rand(size(x)));
    % if r'*r > k
    %     r = k*r./(r'*r);
    % end
    y = x + r;
    L = 0.05*(x-p)'*(x-p);

end