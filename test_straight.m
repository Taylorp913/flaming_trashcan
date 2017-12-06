%Test Striaght Road
close all;

x0 = -30;
y0 = 0;
theta0 = 0.1*pi;
v = 10;
e = 10;

road_width = e*2;
road_init_y = 0;
distance = 350;
step = [1:distance];

E_l = road_init_y + ones(1,distance)*road_width/2;
E_r = road_init_y + ones(1,distance)*road_width/2*-1;
Center = E_l+E_r;

hold on;
plot(step,E_l,'r--');
plot(step,E_r,'r--');
plot(step,Center,'y--');

%ylim([-1*road_width,road_width]);

sim('fig913');

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
plot(move_x,move_y,'.-');
box= [-50 50];
xlim(box+50);
ylim(box);
 

%plot(x,y,'.-');

x_loc_sig = logsout.getElement('x_loc');
y_loc_sig = logsout.getElement('y_loc');
%theta_sig = logsout.getElement('theta');

x_loc = x_loc_sig.Values.Data;
y_loc = y_loc_sig.Values.Data;
%theta = theta_sig.Values.Data;

