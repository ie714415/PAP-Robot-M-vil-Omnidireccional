function Plot_OmniRobot(t,X,Y,Psi)
    %Parameters
    L = 0.165;   l = 0.1;   H = 0.055;   R_w = 0.045;  W_w = 0.05;
    y_top = zeros(1,31)+W_w;    y_bottom = zeros(1,31);
    %Initial points of Platform
    P_top_1_ = [L;-l;R_w+H/2];      P_top_2_ = [L;l;R_w+H/2];       P_top_3_ = [-L;l;R_w+H/2];      P_top_4_ = [-L;-l;R_w+H/2];
    P_bottom_1_ = [L;-l;R_w-H/2];   P_bottom_2_ = [L;l;R_w-H/2];    P_bottom_3_ = [-L;l;R_w-H/2];   P_bottom_4_ = [-L;-l;R_w-H/2];
    P_side_1_1_ = [L;l;R_w-H/2];    P_side_1_2_ = [L;l;R_w+H/2];    P_side_1_3_ = [-L;l;R_w+H/2];   P_side_1_4_ = [-L;l;R_w-H/2];
    P_side_2_1_ = [L;-l;R_w-H/2];   P_side_2_2_ = [L;-l;R_w+H/2];   P_side_2_3_ = [-L;-l;R_w+H/2];  P_side_2_4_ = [-L;-l;R_w-H/2];
    P_side_3_1_ = [L;-l;R_w-H/2];   P_side_3_2_ = [L;l;R_w-H/2];    P_side_3_3_ = [L;l;R_w+H/2];    P_side_3_4_ = [L;-l;R_w+H/2];
    P_side_4_1_ = [-L;-l;R_w+H/2];  P_side_4_2_ = [-L;l;R_w+H/2];   P_side_4_3_ = [-L;l;R_w-H/2];   P_side_4_4_ = [-L;-l;R_w-H/2];
    %Initial points of Wheels
    F_R_W_ = [L;-l;R_w];  F_L_W_ = [L;l;R_w];  R_L_W_ = [-L;l;R_w];  R_R_W_ = [-L;-l;R_w];
    [z_circ,x_circ,y_circ] = cylinder(1,30);
    F_R_W = [(x_circ*R_w)+F_R_W_(1);(y_circ*W_w)+F_R_W_(2)-W_w;(z_circ*R_w)+F_R_W_(3)];
    F_L_W = [(x_circ*R_w)+F_L_W_(1);(y_circ*W_w)+F_L_W_(2);(z_circ*R_w)+F_L_W_(3)];
    R_L_W = [(x_circ*R_w)+R_L_W_(1);(y_circ*W_w)+R_L_W_(2);(z_circ*R_w)+R_L_W_(3)];
    R_R_W = [(x_circ*R_w)+R_R_W_(1);(y_circ*W_w)+R_R_W_(2)-W_w;(z_circ*R_w)+R_R_W_(3)];
    
    figure('units','normalized','outerposition',[0 0 1 1]);
    
    for i=1:length(t)
        cla;
        %Rotation Matrix and Translation Vector
        R = [cos(Psi(i)),-sin(Psi(i)),0; sin(Psi(i)),cos(Psi(i)),0; 0,0,1];
        t = [X(i);Y(i);0];
        %Transformed points of Platform
        P_top_1 = R*P_top_1_+t;         P_top_2 = R*P_top_2_+t;         P_top_3 = R*P_top_3_+t;         P_top_4 = R*P_top_4_+t;
        P_bottom_1 = R*P_bottom_1_+t;   P_bottom_2 = R*P_bottom_2_+t;   P_bottom_3 = R*P_bottom_3_+t;   P_bottom_4 = R*P_bottom_4_+t;
        P_side_1_1 = R*P_side_1_1_+t;   P_side_1_2 = R*P_side_1_2_+t;   P_side_1_3 = R*P_side_1_3_+t;   P_side_1_4 = R*P_side_1_4_+t;
        P_side_2_1 = R*P_side_2_1_+t;   P_side_2_2 = R*P_side_2_2_+t;   P_side_2_3 = R*P_side_2_3_+t;   P_side_2_4 = R*P_side_2_4_+t;
        P_side_3_1 = R*P_side_3_1_+t;   P_side_3_2 = R*P_side_3_2_+t;   P_side_3_3 = R*P_side_3_3_+t;   P_side_3_4 = R*P_side_3_4_+t;
        P_side_4_1 = R*P_side_4_1_+t;   P_side_4_2 = R*P_side_4_2_+t;   P_side_4_3 = R*P_side_4_3_+t;   P_side_4_4 = R*P_side_4_4_+t;
        %Transformed points of Wheels
        F_R_W_1 = R*[F_R_W(1,:);F_R_W(3,:);F_R_W(5,:)]+t;   F_R_W_2 = R*[F_R_W(2,:);F_R_W(4,:);F_R_W(6,:)]+t;
        F_L_W_1 = R*[F_L_W(1,:);F_L_W(3,:);F_L_W(5,:)]+t;   F_L_W_2 = R*[F_L_W(2,:);F_L_W(4,:);F_L_W(6,:)]+t;
        R_L_W_1 = R*[R_L_W(1,:);R_L_W(3,:);R_L_W(5,:)]+t;   R_L_W_2 = R*[R_L_W(2,:);R_L_W(4,:);R_L_W(6,:)]+t;
        R_R_W_1 = R*[R_R_W(1,:);R_R_W(3,:);R_R_W(5,:)]+t;   R_R_W_2 = R*[R_R_W(2,:);R_R_W(4,:);R_R_W(6,:)]+t;
        %Transformed points of cicles 
        F_R_W_t = R*[(x_circ(1,:)*R_w)+F_R_W_(1);y_top+F_R_W_(2)-W_w;(z_circ(1,:)*R_w)+F_R_W_(3)]+t;
        F_R_W_b = R*[(x_circ(2,:)*R_w)+F_R_W_(1);y_bottom+F_R_W_(2)-W_w;(z_circ(2,:)*R_w)+F_R_W_(3)]+t;
        F_L_W_t = R*[(x_circ(1,:)*R_w)+F_L_W_(1);y_top+F_L_W_(2);(z_circ(1,:)*R_w)+F_L_W_(3)]+t;
        F_L_W_b = R*[(x_circ(2,:)*R_w)+F_L_W_(1);y_bottom+F_L_W_(2);(z_circ(2,:)*R_w)+F_L_W_(3)]+t;
        R_L_W_t = R*[(x_circ(1,:)*R_w)+R_L_W_(1);y_top+R_L_W_(2);(z_circ(1,:)*R_w)+R_L_W_(3)]+t;
        R_L_W_b = R*[(x_circ(2,:)*R_w)+R_L_W_(1);y_bottom+R_L_W_(2);(z_circ(2,:)*R_w)+R_L_W_(3)]+t;
        R_R_W_t = R*[(x_circ(1,:)*R_w)+R_R_W_(1);y_top+R_R_W_(2)-W_w;(z_circ(1,:)*R_w)+R_R_W_(3)]+t;
        R_R_W_b = R*[(x_circ(2,:)*R_w)+R_R_W_(1);y_bottom+R_R_W_(2)-W_w;(z_circ(2,:)*R_w)+R_R_W_(3)]+t;
        
        hold on;
   
        %Inertial Frame
        line([0 0.2],[0 0],[0 0],'Color',[1 0.1 0],'LineWidth',3,'LineStyle',':'); %X0
        line([0 0],[0 0.2],[0 0],'Color',[0.1 1 0],'LineWidth',3,'LineStyle',':'); %Y0
        line([0 0],[0 0],[0 0.2],'Color',[0 0.1 1],'LineWidth',3,'LineStyle',':'); %Z0        
        %Base
        fill3([max(X)+L;max(X)+L;min(X)-L;min(X)-L],[min(Y)-L;max(Y)+L;max(Y)+L;min(Y)-L],[0;0;0;0],0.9*[1 1 1]);
        % Trajectory
        plot3(X(1:i),Y(1:i),0*Y(1:i),'k--','LineWidth',1);
        %Draw polygons
        fill3([P_top_1(1);P_top_2(1);P_top_3(1);P_top_4(1)],[P_top_1(2);P_top_2(2);P_top_3(2);P_top_4(2)],[P_top_1(3);P_top_2(3);P_top_3(3);P_top_4(3)],0*[1 1 1]);
        fill3([P_bottom_1(1);P_bottom_2(1);P_bottom_3(1);P_bottom_4(1)],[P_bottom_1(2);P_bottom_2(2);P_bottom_3(2);P_bottom_4(2)],[P_bottom_1(3);P_bottom_2(3);P_bottom_3(3);P_bottom_4(3)],0.6*[1 1 1]);
        fill3([P_side_1_1(1);P_side_1_2(1);P_side_1_3(1);P_side_1_4(1)],[P_side_1_1(2);P_side_1_2(2);P_side_1_3(2);P_side_1_4(2)],[P_side_1_1(3);P_side_1_2(3);P_side_1_3(3);P_side_1_4(3)],0.6*[1 1 1]);
        fill3([P_side_2_1(1);P_side_2_2(1);P_side_2_3(1);P_side_2_4(1)],[P_side_2_1(2);P_side_2_2(2);P_side_2_3(2);P_side_2_4(2)],[P_side_2_1(3);P_side_2_2(3);P_side_2_3(3);P_side_2_4(3)],0.6*[1 1 1]);
        fill3([P_side_3_1(1);P_side_3_2(1);P_side_3_3(1);P_side_3_4(1)],[P_side_3_1(2);P_side_3_2(2);P_side_3_3(2);P_side_3_4(2)],[P_side_3_1(3);P_side_3_2(3);P_side_3_3(3);P_side_3_4(3)],0.6*[1 1 1]);
        fill3([P_side_4_1(1);P_side_4_2(1);P_side_4_3(1);P_side_4_4(1)],[P_side_4_1(2);P_side_4_2(2);P_side_4_3(2);P_side_4_4(2)],[P_side_4_1(3);P_side_4_2(3);P_side_4_3(3);P_side_4_4(3)],0.6*[1 1 1]);
        %Draw Cylinders
        mesh(gca,[F_R_W_1(1,:);F_R_W_2(1,:)],[F_R_W_1(2,:);F_R_W_2(2,:)],[F_R_W_1(3,:);F_R_W_2(3,:)],'FaceColor',0*[1 1 1],'EdgeColor',0*[1 1 1]); 
        fill3(gca,F_R_W_t(1,:),F_R_W_t(2,:),F_R_W_t(3,:),[0.8 0 1]); %Morado
        fill3(gca,F_R_W_b(1,:),F_R_W_b(2,:),F_R_W_b(3,:),[0.1 1 0]); %verde 
        mesh(gca,[F_L_W_1(1,:);F_L_W_2(1,:)],[F_L_W_1(2,:);F_L_W_2(2,:)],[F_L_W_1(3,:);F_L_W_2(3,:)],'FaceColor',0*[1 1 1],'EdgeColor',0*[1 1 1]);
        fill3(gca,F_L_W_t(1,:),F_L_W_t(2,:),F_L_W_t(3,:),[0.8 0 1]);
        fill3(gca,F_L_W_b(1,:),F_L_W_b(2,:),F_L_W_b(3,:),[0.1 1 0]);
        mesh(gca,[R_L_W_1(1,:);R_L_W_2(1,:)],[R_L_W_1(2,:);R_L_W_2(2,:)],[R_L_W_1(3,:);R_L_W_2(3,:)],'FaceColor',0*[1 1 1],'EdgeColor',0*[1 1 1]);
        fill3(gca,R_L_W_t(1,:),R_L_W_t(2,:),R_L_W_t(3,:),[0.8 0 1]);
        fill3(gca,R_L_W_b(1,:),R_L_W_b(2,:),R_L_W_b(3,:),[0.1 1 0]);
        mesh(gca,[R_R_W_1(1,:);R_R_W_2(1,:)],[R_R_W_1(2,:);R_R_W_2(2,:)],[R_R_W_1(3,:);R_R_W_2(3,:)],'FaceColor',0*[1 1 1],'EdgeColor',0*[1 1 1]);
        fill3(gca,R_R_W_t(1,:),R_R_W_t(2,:),R_R_W_t(3,:),[0.8 0 1]);
        fill3(gca,R_R_W_b(1,:),R_R_W_b(2,:),R_R_W_b(3,:),[0.1 1 0]);

        hold off;   grid on;    axis('equal',[min(X)-L max(X)+L min(Y)-L max(Y)+L -1.2 1.2]); view([1,1,1]);
        pause(0.01);
    end
end