function RobotMovilOmnidireccional_Control(tspan, x0, Ke_, n_, A_)
    global Ke n A
    Ke = Ke_; 
    n = n_ + 1.25;
    A = A_;
 
    [t, X] = ode45(@RobotMovilOmnidireccional_Control_sys, tspan, x0);
    
    %Graficas
    figure; subplot(3,1,1); plot(t,X(:,1)); title('SALIDA 1'); grid;
            subplot(3,1,2); plot(t,X(:,2)); title('SALIDA 2'); grid;
            subplot(3,1,3); plot(t,X(:,3),t,A*sin(t*n),'--'); title('SALIDA Y REFERENCIA 3'); grid;
end

function dX = RobotMovilOmnidireccional_Control_sys(t, X)
    global Ke n A
    %Parámetros del sistema
    Rw = 0.045; L = 0.165; l = 0.1; time = t*n;
    %Estados 
    psi = X(3); 
    %Matrices del Sistema 
    B = (Rw/4)*[cos(psi)+sin(psi) cos(psi)-sin(psi) cos(psi)-sin(psi) cos(psi)+sin(psi);
                sin(psi)-cos(psi) sin(psi)+cos(psi) sin(psi)+cos(psi) sin(psi)-cos(psi);
                -1/(L+l) 1/(L+l) -1/(L+l) 1/(L+l)];
    C = [0 0 1];
    % Referencia y sus derivadas
    psi_ref = A*sin(time);
    dpsi_ref = (A*n)*cos(time);
   
    %Controlador
    e = psi_ref - C*X;
    u = ((L+l)/Rw)*(dpsi_ref - Ke*e);
    U = [-u; u; -u; u];
    %ODE's 
    dX = B*U;
end 