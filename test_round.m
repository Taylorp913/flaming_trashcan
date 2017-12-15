%test round
%Test Striaght Road
close all;

x0 = -30;
y0 = 35;
theta0 = pi*0.4;
road_radius = 50;
v = 35;
e = 5;

dt = 0.01;
runtime = 16;


road_width = e*2;
road_init_y = 0;
distance = 350;
step = [1:distance];

centers = [0 0; 0 0;0 0];
radii = [road_radius+e
         road_radius
         road_radius-e
         (road_radius+(e*1.5))
         (road_radius-(e*1.5))
         ];
     
viscircles(centers(1,:),radii(1),'LineStyle','-','Color','b');
viscircles(centers(2,:),radii(2),'LineStyle','--','Color','y');
viscircles(centers(3,:),radii(3),'LineStyle','-','Color','b');
viscircles(centers(3,:),radii(4),'LineStyle',':','Color','g');
viscircles(centers(3,:),radii(5),'LineStyle',':','Color','g');


E_l = road_init_y + ones(1,distance)*road_width/2;
E_r = road_init_y + ones(1,distance)*road_width/2*-1;
Center = E_l+E_r;

hold on;
% plot(step,E_l,'r--');%outer circle
% plot(step,E_r,'r--');
% plot(step,Center,'y--');

%ylim([-1*road_width,road_width]);
sfc('coder_options','forceDebugOff',1);

sim('fig913_cont');

%logsout.plot

x_sig = logsout.getElement('x');
y_sig = logsout.getElement('y');
%theta_sig = logsout.getElement('theta');

x = x_sig.Values.Data;
y = y_sig.Values.Data;
%theta = theta_sig.Values.Data;

move_y = [1:length(y)];
move_x = [1:length(y)];
 
move_y(1) = y(1);
move_x(1) = x(1);
for i=[2:length(y)]
 move_y(i)= move_y(i-1)+y(i); 
 move_x(i)= move_x(i-1)+x(i); 
end

%close all;

% for i=1:length(x)
%     plot(x(i),y(i),'b')
%     %axis([-5 5 -5 5])
%     %pause(.00000001)
% end
set(gcf, 'Position', [500, 500, 500,500])
plot(x,y,'r.-');

%box= [-50 50];
%xlim(box+50);
%ylim(box);
%plot(x,y,'.-');



