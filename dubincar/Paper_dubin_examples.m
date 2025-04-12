%%  dubin
clc;
clear all;
close all;
%% Generate a nominal Trajectory

A = @(u,th) [ 0, 0, 0, 1, 0, 0;
            0, 0, 0, 0, 1, 0;
            0, 0, 0, 0, 0, 1;
            0, 0, -u*sin(th), 0, 0, 0;
            0, 0,  u*cos(th), 0, 0, 0;
            0, 0, 0, 0, 0, 0];


B = @(u,th) [  0, 0 ;
            0, 0 ;
            0, 0 ;
            cos(th), 0 ;
            sin(th), 0 ;
            0, 1];


C = [eye(3),zeros(3,3)];

D=[];

dt = 0.1;

th = pi/2;
u = 1;
sys1 = ss(A(u,th),B(u,th),C,D);
sys2 = c2d(sys1,dt);

At = sys2.A;
Bt = sys2.B;
Ct = sys2.C;

n = 6;
m = 2;
l = 3;

N = 150;

x0 = [1,1,-pi,0,0,0]';
Xinit = kron(ones(N,1),x0);
Uinit = zeros(m*(N-1),1);
%
Q = diag([100,100,10, ...
            1000,1000,10])


R = 10*eye(2);

Xdata = reshape(Xinit,n,[]);
iter= 50;
rr=0;

for k = 1:iter
    
    cvx_begin
        variable dU(m*(N-1))
        variable dX(n*(N))
        obj=0;
        for i=1:N-1
            x = Xinit((i-1)*n+1:(i)*n);
            u = Uinit((i-1)*m+1:(i)*m);
            dx = dX((i-1)*n+1:(i)*n);
            du = dU((i-1)*m+1:(i)*m);
            
            r = [0;0;-pi;0;0;0];
            obj = obj + (x+dx-r)'*Q*(x+dx-r) + (u+du)'*R*(u+du);
  
        end
        x = Xinit((i-1)*n+1:(i)*n);
        dx = dX((N-1)*n+1:(N)*n);
        obj = obj + (x+dx-r)'*Q*(x+dx-r);

        minimize(obj )
        subject to

            for i = 1:N-1
                
                th = Xinit((i-1)*n+3);
                u  = Uinit((i-1)*m+1);
                sys1 = ss(A(u,th),B(u,th),C,D);
                sys2 = c2d(sys1,dt);

                At = sys2.A;
                Bt = sys2.B;
                
                dx = dX((i-1)*n+1:(i)*n);
                du = dU((i-1)*m+1:(i)*m);
                % dX((i)*n+1:(i+1)*n) == At*dX((i-1)*n+1:(i)*n) + Bt*dU((i-1)*m+1:(i)*m);
                dX((i)*n+1:(i+1)*n) == At*dx + Bt*du;
                
                % trust region 
                norm(du) <= 5*0.5^rr;
                
                % iteration constraints
                c1=[1,0];
                c2=[0,1];
                c1*du >= 0;
                c2*du >= -pi/6;
                c2*du <= pi/6;

            end

            dX(1:n) == 0;
       
    cvx_end

    norm(dX)
    if norm(dX)<=0.00001
        break
    end
    Xinit = Xinit + dX;
    Uinit = Uinit + dU;
    




    Xdata = [Xdata;reshape(Xinit,n,[])];


end

Xinit0 = Xinit;
Uinit0 = Uinit;

%
xx = reshape(Xinit,n,[]);
uu = reshape(Uinit,m,[]);
%
close all;
figure()
for s = 1:k+1
xx = reshape(Xdata((k-1)*n+1:(k)*n,:),n,[]);
plot(xx(1,:),xx(2,:),'r','LineWidth',2);hold on;
plot(xx(1,:),xx(2,:),'or','LineWidth',2);hold on;
ylabel('y')
xlabel('x')
end

figure()
for s = 1:k+1
subplot(2,2,1)
xx = reshape(Xdata((k-1)*n+1:(k)*n,:),n,[]);
plot(xx(1,:),'r','LineWidth',2);hold on;
plot(xx(2,:),'k','LineWidth',2);hold on;
ylabel('position')
xlabel('time')
subplot(2,2,2)
plot(xx(3,:),'r','LineWidth',2);hold on;
ylabel('orientation')
xlabel('time')

subplot(2,2,3)
xx = reshape(Xdata((k-1)*n+1:(k)*n,:),n,[]);
plot(xx(4,:),'r','LineWidth',2);hold on;
plot(xx(5,:),'k','LineWidth',2);hold on;
ylabel('velocities')
xlabel('time')
subplot(2,2,4)
plot(xx(6,:),'r','LineWidth',2);hold on;
ylabel('omega')
xlabel('time')
end

%% Solving Estimation aware problem for differnet exploration params
Ct1 = [ 1,0,0,0,0,0;
        0,1,0,0,0,0;]
close all;


xdata=[];
N=140

g = 1e1
r=0;

% X_bar0 = Xinit;
% X_bar = X_bar0;
% 
% U_bar0 = u(:);
% U_bar = U_bar0;

s=0;
xdata.(append('x',num2str(s+1))) = xx;

Xinit = Xinit0(1:(N*n));
Uinit = Uinit0(1:((N-1)*m));

Xinit0 = Xinit0(1:(N*n));
Uinit0 = Uinit0(1:((N-1)*m));


data = Xinit;
dxdata = [];
CTdata=[];
OBJdata = [];


% set the exploration variable

for s=5:5:15
    DDx=[];
Xinit = Xinit0(1:(N*n));
Uinit = Uinit0(1:((N-1)*m));
ct=[];
objval=[];
    s
    r=0;
    
    for k = 1:10

    ly_exp=[];
    eps=0.5;
    UU=[];

    
    rr = 10;

    cvx_begin quiet
        variable dU(m*(N-1))
        variable dX(n*(N))
        L=[];
        Kl=[];
        obj=0;

        for i=1:N-1
            dx = dX((i-1)*n+1:(i)*n);
            du = dU((i-1)*m+1:(i)*m);

            xi = Xinit((i-1)*n+1:(i)*n);
            ui = Uinit((i-1)*m+1:(i)*m);

            x = xi+dx;

            lx = phi_set_L_cvx3(Ct1*x);
            kx = phi_set_k_cvx3(Ct1*x);
            L=[L,lx];
            Kl=[Kl,kx];
            
            th = xi(3);
            
            sys1 = ss(A(ui(1),th),B(ui(1),th),C,D);
            sys2 = c2d(sys1,dt);
            At = sys2.A;
            Bt = sys2.B;

            ly = eps*(min(real(eig((At)^i))));
            obj = obj + (ly'*ly*(1 - lx) - 2*kx);
        end       


      
        
        maximize(obj )
        subject to
          
            dX(1:n) == 0;
            dX(end-n:end) == 0;
            

            for i = 1:N-1
                th = Xinit((i-1)*n+3);
                u = Uinit((i-1)*m+1);
                sys1 = ss(A(u(1),th),B(u(1),th),C,D);
                sys2 = c2d(sys1,dt);

                At = sys2.A;
                Bt = sys2.B;
                
                

                dX((i)*n+1:(i+1)*n) ==   At*dX((i-1)*n+1:(i)*n) + Bt*dU((i-1)*m+1:(i)*m);
                
                %
                c1=[1,0];
                c2=[0,1];
                c1*dU((i-1)*m+1:(i)*m) >= 0
                
                c2*dU((i-1)*m+1:(i)*m) >= -pi
                c2*dU((i-1)*m+1:(i)*m) <= pi
                norm(dX((i-1)*n+1:(i)*n)) <= 0.9*0.5^r
                
                % norm(Uinit0((i-1)*m+1:(i)*m) - Uinit((i-1)*m+1:(i)*m) -dU((i-1)*m+1:(i)*m)) <= 0.1
                if i<N-2
                    norm(dU((i-1)*m+2) - dU((i)*m+2)) <= 0.05
                end
            end

    cvx_end
   
    xbar = reshape(Xinit,n,[]);
 
    Xtemp = Xinit + dX;
    Utemp = Uinit + dU;
    xx2 = reshape(Xinit,n,[]);


    ddx = norm(vecnorm(reshape(Xtemp-Xinit0,n,[]),2));
    ddx
    r

    if ddx>1*s
        r = r+1;

    else 
    Xinit = Xinit + dX;
    Uinit = Uinit + dU;
    data=[data,Xinit];
    DDx = [DDx,ddx];
    ct = [ct,cvx_cputime];
    objval= [objval,cvx_optval]
    end

    if r>10
        break
    end


    end

xx2 = reshape(Xinit,n,[]);
uu2 = reshape(Uinit,m,[]);

UU = [UU;uu];

dd = norm(vecnorm(xx2 - xbar,2),Inf);


xdata.(append('x',num2str(s+1))) = xx2;
udata.(append('u',num2str(s+1))) = uu2;
dxdata.(append('x',num2str(s))) = DDx;
CTdata.(append('x',num2str(s))) = ct;
OBJdata.(append('x',num2str(s))) = objval;

end 




%% Trajectory Evaluation
yn = fieldnames(xdata);
x= zeros(n,N);
u= zeros(m,N);
y= zeros(2,N);
x(:,1) = Xinit0(1:n); 
e = zeros(n,N);

E1=[];
Edata=[];
E2data=[];
E = [];

for k = 1:length(yn)
% for k = 1:5

xx = xdata.(yn{k});
E2  = [];
E=[];
NN = 5000;

    for s = 1:NN

    e = zeros(n,N);
    e2 = zeros(n,N-1);

        for i=1:N-1

            x1 = xx(:,i);
            u1 = uu(:,i);
            th = xx(3);  
            sys1 = ss(A(u1(1),th),B(u1(1),th),C,D);
            sys2 = c2d(sys1,dt);

            At = sys2.A;
            Bt = sys2.B;
            p=[0.9,0.9,0.95,0.99,0.99,0.995];
            L = place(At',Ct1',p)';
            ey = Ct1*x1 - phi2(Ct1*x1);

            e(:,i+1) = (At - L*Ct1)*e(:,i)+ L*ey;
            e2(:,i) = norm((At - L*Ct1)*e(:,i)+ L*ey);
    

        end

        E = [E;e];
        E2 = [E2;e2];
    end
    Edata.(append('x',num2str(k))) = E;
    E2data.(append('e',num2str(k))) = E2;

end






%%
close all;

%plotting tools
rmax =10;
 r = linspace(0, 2.5, rmax);                               % Define Radius & Radius Gradient Vector
a = linspace(0, 2*pi, 150);                             % Angles (Radians)
[Rr,Aa] = ndgrid(r, a);                                   % Create Grid
Z1 = -Rr;                                                 % Create Gradient Matrix
[X1,Y1,Z1] = pol2cart(Aa,Rr,Z1);                              % Convert To Cartesian


figure()
xlabel('x','FontSize',14);
surf(X1+1, Y1, Z1);hold on
for j =1:1:length(yn)
    % for j =1:5
    xx = xdata.(yn{j});
    % plot(xx(1,:),xx(2,:),'Color',[(1-0.75*1/j),0*(1-1*1/j),(1-0.99*1/j)],'LineWidth',2*(1-0.1*1/j));hold on;
    plot(xx(1,:),xx(2,:),'Color',[2*0.125*(j-1),0,2*0.125*(j-1)],'LineWidth',2*(1-0.1*1/j));hold on;

end
% plot(xx(1,1),xx(2,1),'ro','Linewidth',3);
% plot(xx(1,1),xx(2,1),'ko');
% plot(xx(1,end),xx(2,end),'bo');
% % plot(x(1,:),x(2,:),'b','LineWidth',2);hold on;
% plot(x(1,1),x(2,1),'ko');
% plot(x(1,end),x(2,end),'bo');
plot(1,1,'ko','LineWidth',3)
plot(1,0,'ro','LineWidth',3)
plot(0,0,'go','LineWidth',3)
xlabel('x','FontSize',18,'FontWeight','bold');
ylabel('y','FontSize',18,'FontWeight','bold');
view(0, 90)
axis([-0.5 1.5 -0.5 1.5])
colormap(bone)
shading('interp')
legend("","\gamma = 0","\gamma = 10","\gamma = 20","\gamma = 30","\gamma = 35","","","","",'Interpreter','tex','FontSize',14,'FontWeight','Bold')
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Times','fontsize',18)


% subplot(1,2,2)
% plot(Em1,'b','LineWidth',3);hold on
% plot(Em2,'k','LineWidth',3);
% 
% plot(Es1,'--b','LineWidth',3);hold on
% plot(Es2,'--k','LineWidth',3);
% ylabel('Tracking Error','FontSize',14);
% xlabel('Time','FontSize',14);
e = fieldnames(E2data);
figure()
for j =1:1:length(yn)
    % for j =1:5
    xx = xdata.(yn{j});

    % subplot(1,2,1)
    % plot(xx(1,:),xx(2,:),'Color',[1-0.9*1/j,0,1-0.9*1/j],'LineWidth',2*(1-0.1*1/j));hold on;


    E = E2data.(e{j});

    Ev = E2data.(e{j});
    % Av=[];
    % for k = 1:N-1
    %     en = reshape(Ev(:,k),n,[]);
    %     Av = [Av;trace(cov(en))];
    % end

    Em = max(E,[],1);
    Es = mean(E,1);
    
    
    

    plot(Em,'LineStyle','-','Color',[2*0.125*(j-1),0,2*0.125*(j-1)],'LineWidth',2.5);hold on
    % area(Em,'FaceColor',[1-0.9*1/j,0,1-0.9*1/j]);hold on
    % plot(Em2,'k','LineWidth',3);


    plot(Es,'LineStyle','--','Color',[0.125*(j-1),0,0.125*(j-1)],'LineWidth',3.5);hold on

    % plot(Es2,'--k','LineWidth',3);
    ylabel('Tracking Error','FontSize',14);
% xlabel('Time','FontSize',14);
xlabel('Time','FontSize',14,'FontWeight','bold');
ylabel('Tracking Error Norm ','FontSize',14,'FontWeight','bold');

end
legend("","","","","","","","","max error","mean error",'Interpreter','tex','FontSize',14,'FontWeight','Bold')
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Times','fontsize',14,'FontWeight','bold')
grid on



figure
% 
for j =1:1:length(yn)

% for j =1:5
    xx = xdata.(yn{j});

    % subplot(1,2,1)
    % plot(xx(1,:),xx(2,:),'Color',[1-0.9*1/j,0,1-0.9*1/j],'LineWidth',2*(1-0.1*1/j));hold on;


    E = E2data.(e{j});

    Ev = E2data.(e{j});
    Av=[];
    for k = 1:N-1
        en = reshape(Ev(:,k),n,[]);
        Av = [Av;trace(cov(en'))];
    end

    Em = max(E,[],1);
    Es = mean(E,1);
    
    
plot(Av,'LineStyle','-','Color',[2*0.125*(j-1),0,2*0.125*(j-1)],'LineWidth',3);hold on
xlabel('Time','FontSize',14,'FontWeight','bold');
ylabel('Trace of State Covariance','FontSize',14,'FontWeight','bold');
legend("\gamma = 0","\gamma = 10","\gamma = 20","\gamma = 30","\gamma = 35",'Interpreter','tex','FontSize',14,'FontWeight','Bold')
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Times','fontsize',14,'FontWeight','bold')
grid on
end




%%


close all
e = fieldnames(OBJdata);
figure()
for j =length(e):-1:1
    % for j =1:5
    dx = dxdata.(e{j});
    plot(dx,'Color',[2*0.125*(j-1),0,2*0.125*(j-1)],'LineWidth',2);hold on;
end

xlabel('Iterations','FontSize',18,'FontWeight','bold');
ylabel('Total trajectory deviation','FontSize',14,'FontWeight','bold','Interpreter','tex');
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Times','fontsize',14)
% legend("\gamma = 5","\gamma = 10","\gamma = 15","\gamma = 20","\gamma = 25","\gamma = 30","\gamma = 35","\gamma = 40",'Interpreter','tex','FontSize',14,'FontWeight','Bold')
grid on
% a = get(gca,'XTickLabel');
% set(gca,'XTickLabel',a,'FontName','Times','fontsize',18)


figure()
% for j =length(e):-1:1
for j =1:1:length(e)    
    % for j =1:5
    ob = OBJdata.(e{j});
    % plot(ob,'Color',[(1-1*1/j),0,(1-1*1/j)],'LineWidth',2*(1-0.1*1/j));hold on;
    plot(ob,'Color',[2*0.125*(j-1),0,2*0.125*(j-1)],'LineWidth',2);hold on;
end

xlabel('Iterations','FontSize',14,'FontWeight','bold');
ylabel('Objective function','FontSize',14,'FontWeight','bold');
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Times','fontsize',14)
legend("\gamma = 5","\gamma = 10","\gamma = 15","\gamma = 20","\gamma = 25","\gamma = 30","\gamma = 35","\gamma = 40",'Interpreter','tex','FontSize',14,'FontWeight','Bold')
grid on
% a = get(gca,'XTickLabel');
% set(gca,'XTickLabel',a,'FontName','Times','fontsize',18)


