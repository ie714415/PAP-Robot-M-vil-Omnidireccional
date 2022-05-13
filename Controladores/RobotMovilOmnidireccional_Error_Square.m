function RobotMovilOmnidireccional_Error_Square(tspan, x0, Ke_, c, side, squares)
    global Ke cx cy s n
    Ke = Ke_; cx = c(1); cy = c(2); s = side; n = squares; 
 
    [t, X] = ode45(@RobotMovilOmnidireccional_Error_Square_sys, tspan, x0);
    
    %Graficas
    figure; subplot(3,1,1); plot(t,X(:,1)); title('SALIDA 1'); grid;
            subplot(3,1,2); plot(t,X(:,2)); title('SALIDA 2'); grid;
            subplot(3,1,3); plot(t,X(:,3)); title('SALIDA 3'); grid;
            
    Plot_OmniRobot(t,X(:,1),X(:,2),X(:,3));
end

function dX = RobotMovilOmnidireccional_Error_Square_sys(t, X)
    global Ke cx cy s n
    %Parámetros del sistema
    Rw = 0.045; L = 0.165; l = 0.1; time = mod(t*n,360);
    %Estados 
    psi = X(3); 
    %Matrices del Sistema 
    B = (Rw/4)*[cos(psi)+sin(psi) cos(psi)-sin(psi) cos(psi)-sin(psi) cos(psi)+sin(psi);
                sin(psi)-cos(psi) sin(psi)+cos(psi) sin(psi)+cos(psi) sin(psi)-cos(psi);
                -1/(L+l) 1/(L+l) -1/(L+l) 1/(L+l)];
    C = [1 0 0; 0 1 0; 0 0 1];
    % Referencia y sus derivadas
    psi_ref = (t*pi)/180;
    dpsi_ref = pi/180;
    
    if(time >= 0 && time <= 45)
        x_ref = cx + s/2;                           y_ref = (time*(cy + s/2))/45;
        dx_ref = 0;                                 dy_ref = (cy + s/2)/45;
    elseif(time > 45 && time <= 135)
        x_ref = cx + s - (time*(cx + s))/90;        y_ref = cy + s/2;
        dx_ref = -(cx + s)/90;                      dy_ref = 0;
    elseif(time > 135 && time <= 225)
        x_ref = -(cx + s/2);                        y_ref = cy + 2*s - (time*(cy + s))/90;
        dx_ref = 0;                                 dy_ref = -(cy + s)/90;
    elseif(time > 225 && time <= 315)
        x_ref = -(cx + 3*s) + (time*(cx + s))/90;   y_ref = -(cy + s/2);
        dx_ref = cx + s/90;                         dy_ref = 0;
    else 
        x_ref = cx + s/2;                           y_ref = -(cy + 4*s) + (time*(cy + s/2))/45;
        dx_ref = 0;                                 dy_ref = (cy + s/2)/45;
    end 
    %Controlador
    e = [x_ref; y_ref; psi_ref] - C*X;
    U = pinv(B)*([dx_ref; dy_ref; dpsi_ref] - Ke*e);
    %ODE's 
    dX = B*U;
end 