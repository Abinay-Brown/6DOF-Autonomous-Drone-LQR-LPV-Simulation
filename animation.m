clear;
clc;
load('Simulation.mat', "pos", "phi_val", "theta_val", "psi_val",...
    "phi_ref", "theta_ref", "psi_ref", "Xr", "Yr", "Zr", "l");
x = pos(:,1);
y = pos(:,2);
z = pos(:,3);
phi = phi_val;
theta = theta_val;
psi = psi_val;

xmax = 0;
xmin = 0;  
ymax = 0; 
ymin = 0;
zmax = 0;
zmin = 0;

plot3(Xr, Yr, Zr, 'k');
hold on
zoom_factor = 2;
for i = 1:1:length(phi)
    
    if(x(i)>=xmax)
        xmax = x(i);
    end
    if(x(i)<=xmin)
        xmin = x(i);
    end
    
    if(y(i)>=ymax)
        ymax = y(i);
    end
    if(y(i)<=ymin)
        ymin = y(i);
    end

    if(z(i)>=zmax)
        zmax = z(i);
    end
    if(z(i)<=zmin)
        zmin = z(i);
    end

    limitmin = min(xmin,ymin); limitmax = max(xmax,ymax);
    xlim([limitmin-zoom_factor,limitmax+zoom_factor]);
    ylim([limitmin-zoom_factor,limitmax+zoom_factor]);
    zlim([zmin-zoom_factor,zmax+zoom_factor]);
    grid on;
    axis equal;
    R_mat = rot(phi(i), theta(i), psi(i));
    pt1 = R_mat*[-l,0,0]';
    pt2 = R_mat*[0,-l,0]'; 

    if i ~= 1
        delete(h1);
        delete(h2);
        plot3([x(i-1),x(i)],[y(i-1),y(i)],[z(i-1),z(i)],"LineWidth",2, "Color", "g")
    else
        plot3(x(i),y(i),z(i),"LineWidth",2, "Color", "g")
    end    
    hold on;
    h1 = plot3([x(i)+pt1(1),x(i)-pt1(1)],...
               [y(i)+pt1(2),y(i)-pt1(2)],...
               [z(i)+pt1(3),z(i)-pt1(3)], ...
               "LineWidth",3,"Color","r");
    
    hold on;
    h2 = plot3([x(i)+pt2(1),x(i)-pt2(1)], ...
               [y(i)+pt2(2),y(i)-pt2(2)],...
               [z(i)+pt2(3),z(i)-pt2(3)], ...
               "LineWidth",3,"Color","b");

    hold on;
    pause(0.05)
end