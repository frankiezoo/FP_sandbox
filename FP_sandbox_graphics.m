%% plot the 2D view (xz plane) of the flux pinned image geometries
pos0_xz=[r_mag_0(1)-0.005 r_mag_0(3)-0.005 0.01 0.01];
mu0_x = [r_mag_0(1) r_mag_0(1)+0.01*mu_mag_0(1)/norm(mu_mag_0)];
mu0_z = [r_mag_0(3) r_mag_0(3)+0.01*mu_mag_0(3)/norm(mu_mag_0)];

r_m0=r_m.data(1,:);
mu_m0=mu_m.data(1,:);
posm0_xz=[r_m0(1)-0.005 r_m0(3)-0.005 0.01 0.01];
mu_m0_x = [r_m0(1) r_m0(1)+0.01*mu_m0(1)/norm(mu_m0)];
mu_m0_z = [r_m0(3) r_m0(3)+0.01*mu_m0(3)/norm(mu_m0)];

r_f0=r_f.data(1,:);
mu_f0=mu_f.data(1,:);
posf0_xz=[r_f0(1)-0.005 r_f0(3)-0.005 0.01 0.01];
mu_f0_x = [r_f0(1) r_f0(1)+0.01*mu_f0(1)/norm(mu_f0)];
mu_f0_z = [r_f0(3) r_f0(3)+0.01*mu_f0(3)/norm(mu_f0)];

% figure
% rectangle('Position',[-.1 -.1 0.2 0.1],'FaceColor',[.7 .7 .7])
% hold on
% rectangle('Position',pos0_xz,'Curvature',[1 1],'FaceColor',[1 0 0])
% rectangle('Position',posm0_xz,'Curvature',[1 1],'FaceColor',[0 1 0])
% rectangle('Position',posf0_xz,'Curvature',[1 1],'FaceColor',[0 1 1])
% quiver(mu0_x(1),mu0_z(1),mu0_x(2)-mu0_x(1),mu0_z(2)-mu0_z(1),'k') 
% quiver(mu_m0_x(1),mu_m0_z(1),mu_m0_x(2)-mu_m0_x(1),mu_m0_z(2)-mu_m0_z(1),'k') 
% quiver(mu_f0_x(1),mu_f0_z(1),mu_f0_x(2)-mu_f0_x(1),mu_f0_z(2)-mu_f0_z(1),'k') 
% axis([-.1 .1 -.1 .1])

% plot the 2D view (xy plane) of the flux pinned image geometries
pos0_xy=[r_mag_0(1)-0.005 r_mag_0(2)-0.005 0.01 0.01];
mu0_y = [r_mag_0(2) r_mag_0(2)+0.02*mu_mag_0(2)/norm(mu_mag_0)];

posm0_xy=[r_m0(1)-0.005 r_m0(2)-0.005 0.01 0.01];
mu_m0_y = [r_m0(2) r_m0(2)+0.01*mu_m0(2)/norm(mu_m0)];

posf0_xy=[r_f0(1)-0.005 r_f0(2)-0.005 0.01 0.01];
mu_f0_y = [r_f0(2) r_f0(2)+0.01*mu_f0(2)/norm(mu_f0)];

% figure
% rectangle('Position',[-.1 -.1 0.2 0.2],'FaceColor',[.7 .7 .7])
% hold on
% rectangle('Position',pos0_xy,'Curvature',[1 1],'FaceColor',[1 0 0])
% rectangle('Position',posm0_xy,'Curvature',[1 1],'FaceColor',[0 1 0])
% rectangle('Position',posf0_xy,'Curvature',[1 1],'FaceColor',[0 1 1])
% quiver(mu0_x(1),mu0_y(1),mu0_x(2)-mu0_x(1),mu0_y(2)-mu0_y(1),'k') 
% quiver(mu_m0_x(1),mu_m0_y(1),mu_m0_x(2)-mu_m0_x(1),mu_m0_y(2)-mu_m0_y(1),'k') 
% quiver(mu_f0_x(1),mu_f0_y(1),mu_f0_x(2)-mu_f0_x(1),mu_f0_y(2)-mu_f0_y(1),'k') 
% axis([-.1 .1 -.1 .1])
% 
% plot the 2D view (yz plane) of the flux pinned image geometries
pos0_yz=[r_mag_0(2)-0.005 r_mag_0(3)-0.005 0.01 0.01];
mu0_y = [r_mag_0(2) r_mag_0(2)+0.02*mu_mag_0(2)/norm(mu_mag_0)];

posm0_yz=[r_m0(2)-0.005 r_m0(3)-0.005 0.01 0.01];
mu_m0_y = [r_m0(2) r_m0(2)+0.01*mu_m0(2)/norm(mu_m0)];

posf0_yz=[r_f0(2)-0.005 r_f0(3)-0.005 0.01 0.01];
mu_f0_y = [r_f0(2) r_f0(2)+0.01*mu_f0(2)/norm(mu_f0)];
% 
% figure
% rectangle('Position',[-.1 -.1 0.2 0.1],'FaceColor',[.7 .7 .7])
% hold on
% rectangle('Position',pos0_yz,'Curvature',[1 1],'FaceColor',[1 0 0])
% rectangle('Position',posm0_yz,'Curvature',[1 1],'FaceColor',[0 1 0])
% rectangle('Position',posf0_yz,'Curvature',[1 1],'FaceColor',[0 1 1])
% quiver(mu0_y(1),mu0_z(1),mu0_y(2)-mu0_y(1),mu0_z(2)-mu0_z(1),'k') 
% quiver(mu_m0_y(1),mu_m0_z(1),mu_m0_y(2)-mu_m0_y(1),mu_m0_z(2)-mu_m0_z(1),'k') 
% quiver(mu_f0_y(1),mu_f0_z(1),mu_f0_y(2)-mu_f0_y(1),mu_f0_z(2)-mu_f0_z(1),'k') 
% axis([-.1 .1 -.1 .1])

F_f0=F_f.data(1,:);
tau_f0=tau_f.data(1,:);
U_f0=U_f.data(1,:);

F_m0=F_m.data(1,:);
tau_m0=tau_m.data(1,:);
U_m0=U_m.data(1,:);

F_tot0=F_f0+F_m0;
tau_tot0=tau_f0+tau_m0;

%% plot position of magnet, mobile image, and frozen image
figure
scatter3(r_mag_0(1),r_mag_0(2),r_mag_0(3),300,[1 0 0],'filled')
hold on
scatter3(r_m0(1),r_m0(2),r_m0(3),300,[0 1 0],'filled')
scatter3(r_f0(1),r_f0(2),r_f0(3),300,[0 1 1],'filled')
line([0 .02],[0 0],[0 0],'LineWidth',2,'Color',[1 0 1])
line([0 0],[0 .02],[0 0],'LineWidth',2,'Color',[0 0 1])
line([0 0],[0 0],[0 .02],'LineWidth',2,'Color',[0 0 0])
line(mu0_x,mu0_y,mu0_z,'LineWidth',2,'Color',[1 0 0])
line(mu_m0_x,mu_m0_y,mu_m0_z,'LineWidth',2,'Color',[0 1 0])
line(mu_f0_x,mu_f0_y,mu_f0_z,'LineWidth',2,'Color',[0 1 1])
[X,Y] = meshgrid(-.1:.005:.1);
M=mesh(X,Y,zeros(length(X),length(Y)));
set(M,'facealpha',0)
set(M,'edgecolor',[.7 .7 .7])
axis(.5*[-.1 .1 -.1 .1 -.1 .1])
view(5,10);
legend('magnet','mobile','frozen','x','y','z')

%% plot force and torque snapshot
figure 
scatter3(r_mag_0(1),r_mag_0(2),r_mag_0(3),300,[1 0 0],'filled')
hold on
scatter3(r_m0(1),r_m0(2),r_m0(3),300,[0 1 0],'filled')
scatter3(r_f0(1),r_f0(2),r_f0(3),300,[0 1 1],'filled')

F_m0_x = [r_mag_0(1) r_mag_0(1)+0.02*F_m0(1)/norm(F_m0)];
F_m0_y = [r_mag_0(2) r_mag_0(2)+0.02*F_m0(2)/norm(F_m0)];
F_m0_z = [r_mag_0(3) r_mag_0(3)+0.02*F_m0(3)/norm(F_m0)];
tau_m0_x = [r_mag_0(1) r_mag_0(1)+0.02*tau_m0(1)/norm(tau_m0)];
tau_m0_y = [r_mag_0(2) r_mag_0(2)+0.02*tau_m0(2)/norm(tau_m0)];
tau_m0_z = [r_mag_0(3) r_mag_0(3)+0.02*tau_m0(3)/norm(tau_m0)];
F_f0_x = [r_mag_0(1) r_mag_0(1)+0.02*F_f0(1)/norm(F_f0)];
F_f0_y = [r_mag_0(2) r_mag_0(2)+0.02*F_f0(2)/norm(F_f0)];
F_f0_z = [r_mag_0(3) r_mag_0(3)+0.02*F_f0(3)/norm(F_f0)];
tau_f0_x = [r_mag_0(1) r_mag_0(1)+0.02*tau_f0(1)/norm(tau_f0)];
tau_f0_y = [r_mag_0(2) r_mag_0(2)+0.02*tau_f0(2)/norm(tau_f0)];
tau_f0_z = [r_mag_0(3) r_mag_0(3)+0.02*tau_f0(3)/norm(tau_f0)];
F_tot0_x = [r_mag_0(1) r_mag_0(1)+0.02*F_tot0(1)/norm(F_tot0)];
F_tot0_y = [r_mag_0(2) r_mag_0(2)+0.02*F_tot0(2)/norm(F_tot0)];
F_tot0_z = [r_mag_0(3) r_mag_0(3)+0.02*F_tot0(3)/norm(F_tot0)];
tau_tot0_x = [r_mag_0(1) r_mag_0(1)+0.02*tau_tot0(1)/norm(tau_tot0)];
tau_tot0_y = [r_mag_0(2) r_mag_0(2)+0.02*tau_tot0(2)/norm(tau_tot0)];
tau_tot0_z = [r_mag_0(3) r_mag_0(3)+0.02*tau_tot0(3)/norm(tau_tot0)];

line(F_m0_x,F_m0_y,F_m0_z,'LineWidth',2,'Color',[0 1 0])
line(F_f0_x,F_f0_y,F_f0_z,'LineWidth',2,'Color',[0 1 1])
line(F_tot0_x,F_tot0_y,F_tot0_z,'LineWidth',2,'Color',[0 0 0])
axis(.5*[-.1 .1 -.1 .1 -.1 .1])

%% animate the magnet and images
mu_x = zeros(size(r_mag.data,1),2);
mu_y = zeros(size(r_mag.data,1),2);
mu_z = zeros(size(r_mag.data,1),2);
mu_m_x = zeros(size(r_mag.data,1),2);
mu_m_y = zeros(size(r_mag.data,1),2);
mu_m_z = zeros(size(r_mag.data,1),2);
mu_f_x = zeros(size(r_mag.data,1),2);
mu_f_y = zeros(size(r_mag.data,1),2);
mu_f_z = zeros(size(r_mag.data,1),2);
F_m_x = zeros(size(r_mag.data,1),2);
F_m_y = zeros(size(r_mag.data,1),2);
F_m_z = zeros(size(r_mag.data,1),2);
tau_m_x = zeros(size(r_mag.data,1),2);
tau_m_y = zeros(size(r_mag.data,1),2);
tau_m_z = zeros(size(r_mag.data,1),2);
F_f_x = zeros(size(r_mag.data,1),2);
F_f_y = zeros(size(r_mag.data,1),2);
F_f_z = zeros(size(r_mag.data,1),2);
tau_f_x = zeros(size(r_mag.data,1),2);
tau_f_y = zeros(size(r_mag.data,1),2);
tau_f_z = zeros(size(r_mag.data,1),2);
F_tot_x = zeros(size(r_mag.data,1),2);
F_tot_y = zeros(size(r_mag.data,1),2);
F_tot_z = zeros(size(r_mag.data,1),2);
tau_tot_x = zeros(size(r_mag.data,1),2);
tau_tot_y = zeros(size(r_mag.data,1),2);
tau_tot_z = zeros(size(r_mag.data,1),2);

F_tot=F_f.data+F_m.data;
tau_tot=tau_f.data+tau_m.data;

for i = 1:size(r_mag.data,1)
    mu_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.01*mu_mag.data(i,1)/norm(mu_mag.data(i,:))];
    mu_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.01*mu_mag.data(i,2)/norm(mu_mag.data(i,:))];
    mu_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.01*mu_mag.data(i,3)/norm(mu_mag.data(i,:))];
    mu_m_x(i,:) = [r_m.data(i,1) r_m.data(i,1)+0.01*mu_m.data(i,1)/norm(mu_m.data(i,:))];
    mu_m_y(i,:) = [r_m.data(i,2) r_m.data(i,2)+0.01*mu_m.data(i,2)/norm(mu_m.data(i,:))];
    mu_m_z(i,:) = [r_m.data(i,3) r_m.data(i,3)+0.01*mu_m.data(i,3)/norm(mu_m.data(i,:))];
    mu_f_x(i,:) = [r_f.data(i,1) r_f.data(i,1)+0.01*mu_f.data(i,1)/norm(mu_f.data(i,:))];
    mu_f_y(i,:) = [r_f.data(i,2) r_f.data(i,2)+0.01*mu_f.data(i,2)/norm(mu_f.data(i,:))];
    mu_f_z(i,:) = [r_f.data(i,3) r_f.data(i,3)+0.01*mu_f.data(i,3)/norm(mu_f.data(i,:))];
    F_m_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*F_m.data(i,1)/norm(F_m.data(i,:))];
    F_m_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*F_m.data(i,2)/norm(F_m.data(i,:))];
    F_m_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*F_m.data(i,3)/norm(F_m.data(i,:))];
    tau_m_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*tau_m.data(i,1)/norm(tau_m.data(i,:))];
    tau_m_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*tau_m.data(i,2)/norm(tau_m.data(i,:))];
    tau_m_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*tau_m.data(i,3)/norm(tau_m.data(i,:))];
    F_f_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*F_f.data(i,1)/norm(F_f.data(i,:))];
    F_f_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*F_f.data(i,2)/norm(F_f.data(i,:))];
    F_f_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*F_f.data(i,3)/norm(F_f.data(i,:))];
    tau_f_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*tau_f.data(i,1)/norm(tau_f.data(i,:))];
    tau_f_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*tau_f.data(i,2)/norm(tau_f.data(i,:))];
    tau_f_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*tau_f.data(i,3)/norm(tau_f.data(i,:))];
    F_tot_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*F_tot(i,1)/norm(F_tot(i,:))];
    F_tot_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*F_tot(i,2)/norm(F_tot(i,:))];
    F_tot_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*F_tot(i,3)/norm(F_tot(i,:))];
    tau_tot_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*tau_tot(i,1)/norm(tau_tot(i,:))];
    tau_tot_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*tau_tot(i,2)/norm(tau_tot(i,:))];
    tau_tot_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*tau_tot(i,3)/norm(tau_tot(i,:))];
end

figure
for time = 1:size(r_mag.data,1)
    %magnets
    scatter3(r_mag.data(time,1),r_mag.data(time,2),r_mag.data(time,3),...
        300,[1 0 0],'filled');
    hold on
    scatter3(r_m.data(time,1),r_m.data(time,2),r_m.data(time,3),...
        300,[0 1 0],'filled');
    scatter3(r_f.data(time,1),r_f.data(time,2),r_f.data(time,3),...
        300,[0 1 1],'filled');
    line([0 .02],[0 0],[0 0],'LineWidth',2,'Color',[1 0 1])
    line([0 0],[0 .02],[0 0],'LineWidth',2,'Color',[0 0 1])
    line([0 0],[0 0],[0 .02],'LineWidth',2,'Color',[0 0 0])
    line(mu_x(time,:),mu_y(time,:),mu_z(time,:),'LineWidth',2,'Color',[1 0 0])
    line(mu_m_x(time,:),mu_m_y(time,:),mu_m_z(time,:),'LineWidth',2,'Color',[0 1 0])
    line(mu_f_x(time,:),mu_f_y(time,:),mu_f_z(time,:),'LineWidth',2,'Color',[0 1 1])
    hold off
    axis(.2*[-.1 .1 -.1 .1 -.1 .1])
    drawnow
end

%% animate the magnet and forces
figure
for time = 1:size(r_mag.data,1)
    %magnets
    scatter3(0,0,0,300,[1 0 0],'filled');
    hold on
%     scatter3(r_m.data(time,1),r_m.data(time,2),r_m.data(time,3),...
%         300,[0 1 0],'filled');
%     scatter3(r_f.data(time,1),r_f.data(time,2),r_f.data(time,3),...
%         300,[0 1 1],'filled');
%     line([0 .02],[0 0],[0 0],'LineWidth',2,'Color',[1 0 1])
%     line([0 0],[0 .02],[0 0],'LineWidth',2,'Color',[0 0 1])
%     line([0 0],[0 0],[0 .02],'LineWidth',2,'Color',[0 0 0])
    line([0 F_tot(time,1)],[0 F_tot(time,2)],[0 F_tot(time,3)],'LineWidth',2,'Color',[1 0 0])
    line([0 F_m.data(time,1)],[0 F_m.data(time,2)],[0 F_m.data(time,3)],'LineWidth',2,'Color',[0 1 0])
    line([0 F_f.data(time,1)],[0 F_f.data(time,2)],[0 F_f.data(time,3)],'LineWidth',2,'Color',[0 1 1])
    axis(.1*[-.1 .1 -.1 .1 -.1 .1])
    hold off
    drawnow
end
%% animate the magnet and forces
figure
for time = 1:size(r_mag.data,1)
    %magnets
    scatter3(0,0,0,300,[1 0 0],'filled');
    hold on
%     scatter3(r_m.data(time,1),r_m.data(time,2),r_m.data(time,3),...
%         300,[0 1 0],'filled');
%     scatter3(r_f.data(time,1),r_f.data(time,2),r_f.data(time,3),...
%         300,[0 1 1],'filled');
%     line([0 .02],[0 0],[0 0],'LineWidth',2,'Color',[1 0 1])
%     line([0 0],[0 .02],[0 0],'LineWidth',2,'Color',[0 0 1])
%     line([0 0],[0 0],[0 .02],'LineWidth',2,'Color',[0 0 0])
    line([0 tau_tot(time,1)],[0 tau_tot(time,2)],[0 tau_tot(time,3)],'LineWidth',2,'Color',[1 0 0])
    line([0 tau_m.data(time,1)],[0 tau_m.data(time,2)],[0 tau_m.data(time,3)],'LineWidth',2,'Color',[0 1 0])
    line([0 tau_f.data(time,1)],[0 tau_f.data(time,2)],[0 tau_f.data(time,3)],'LineWidth',2,'Color',[0 1 1])
    axis(.1*[-.1 .1 -.1 .1 -.1 .1])
    hold off
    drawnow
end

%% energy

figure
plot(TE)
hold on
plot(U_m,'g')
plot(U_f,'c')
plot(TE.time,TE.data+U_m.data+U_f.data,'k')
xlabel('time [sec]')
ylabel('Energy [J]')
legend('kinetic','mobile','frozen','total')
title('Energy vs time of Flux Pinned system')