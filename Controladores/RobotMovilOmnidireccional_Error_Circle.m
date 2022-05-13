function RobotMovilOmnidireccional_Error_Circle(tspan, x0, Ke_, c, radio, circles)
    global Ke cx cy r n
    Ke = Ke_; cx = c(1); cy = c(2); r = radio; n = circles;
 
    [t, X] = ode45(@RobotMovilOmnidireccional_Error_Circle_sys, tspan, x0);
    
    %Graficas
    figure; subplot(3,1,1); plot(t,X(:,1)); title('SALIDA 1'); grid;
            subplot(3,1,2); plot(t,X(:,2)); title('SALIDA 2'); grid;
            subplot(3,1,3); plot(t,X(:,3)); title('SALIDA 3'); grid;
end

function dX = RobotMovilOmnidireccional_Error_Circle_sys(t, X)
    global Ke cx cy r n
    %Parámetros del sistema
    Rw = 5; L = 10; l = 5; time = (t*pi*n)/180;
    %Estados 
    psi = X(3); 
    %Matrices del Sistema 
    B = (Rw/4)*[cos(psi)+sin(psi) cos(psi)-sin(psi) cos(psi)-sin(psi) cos(psi)+sin(psi);
                sin(psi)-cos(psi) sin(psi)+cos(psi) sin(psi)+cos(psi) sin(psi)-cos(psi);
                -1/(L+l) 1/(L+l) -1/(L+l) 1/(L+l)];
    C = [1 0 0; 0 1 0; 0 0 1];
    % Referencia y sus derivadas
    psi_ref = (t*pi)/180;     x_ref = cx + r*cos(time);            y_ref = cy + r*sin(time); 
    dpsi_ref = pi/180;        dx_ref = -((r*pi)/180)*sin(time);    dy_ref = ((r*pi)/180)*cos(time);  
    %Controlador
    e = [x_ref; y_ref; psi_ref] - C*X;
    U = pinv(B)*([dx_ref; dy_ref; dpsi_ref] - Ke*e);
    %ODE's 
    dX = B*U;
end 