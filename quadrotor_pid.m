
function quadrotor_pid(t,x,x_dot,x_ddot,y,y_dot,y_ddot,z,z_dot,z_ddot,phi,phi_dot,theta,theta_dot,psi,psi_dot)
    % calculate desired state as function of t
    
    
	global w1 w2 w3 w4 b l k g desired_z desired_z_dot desired_y desired_y_dot desired_x desired_x_dot ...
            desired_x_ddot desired_y_ddot desired_z_ddot ...
            K_p_z K_d_z K_p_phi K_d_phi K_p_theta K_d_theta K_p_psi K_d_psi ...
            K_i_z K_i_phi K_i_theta K_i_psi ...
            phi_int_err theta_int_err psi_int_err z_int_err...
            w1_store w2_store w3_store w4_store t_store

    x_err = desired_x(t) - x;
    x_dot_err = desired_x_dot(t) - x_dot;
    y_err = desired_y(t) - y;
    y_dot_err = desired_y_dot(t) - y_dot;
    z_err = desired_z(t) - z;
    z_dot_err = desired_z_dot(t) - z_dot;

    desired_phi = 0;
    desired_phi_dot = 0;
    desired_theta = 0;
    desired_theta_dot = 0;
    desired_psi = 0;
    desired_psi_dot = 0;
%     % calculate the desired angles and their time derivatives
%     desired_phi = K_p_phi*y_err + K_d_phi*y_dot_err;
%     desired_phi_dot = K_p_phi*y_dot_err + K_d_phi*(desired_y_ddot(t) - y_ddot);
%     desired_theta = K_p_theta*x_err + K_d_theta*x_dot_err;
%     desired_theta_dot = K_p_theta*x_dot_err + K_d_theta*(desired_x_ddot(t) - x_ddot);
%     desired_psi = K_p_psi*z_err + K_d_psi*z_dot_err;
%     desired_psi_dot = K_p_psi*z_dot_err + K_d_psi*(desired_z_ddot(t) - z_ddot);
    
    
    phi_err = desired_phi - phi;
    phi_dot_err = desired_phi_dot - phi_dot;
    theta_err = desired_theta - theta;
    theta_dot_err = desired_theta_dot - theta_dot;
    psi_err = desired_psi - psi;
    psi_dot_err = desired_psi_dot - psi_dot;
    
    if t <= .001
        phi_int_err = 0;
        theta_int_err = 0;
        psi_int_err = 0;
        z_int_err = 0;
    end
    
    phi_int_err = phi_int_err + phi_err;
    theta_int_err = theta_int_err + theta_err;
    psi_int_err = psi_int_err + psi_err;
    z_int_err = z_int_err + z_err
    
    T = RotB2I(phi, theta, psi)*[0;0;g] + z_err*K_p_z + z_dot_err*K_d_z + z_int_err*K_i_z;
    T(3) = max([T(3) 0]);
    tau_phi = phi_err*K_p_phi + phi_dot_err*K_d_phi + phi_int_err*K_i_phi;
    tau_theta = theta_err*K_p_theta + theta_dot_err*K_d_theta + theta_int_err*K_i_theta;
    tau_psi = psi_err*K_p_psi + psi_dot_err*K_d_psi + psi_int_err*K_i_psi;
    
    
    tau = [T(3) tau_phi tau_theta tau_psi]';
    
    % mixer matrix
    M = [k k k k ; 0 -l*k 0 l*k; -l*k 0 l*k 0; b -b b -b];
    
    
    w_squared = M\tau;
    t_store = [t_store t];
    w1 = sqrt(w_squared(1));
    w1_store = [w1_store w1];
    w2 = sqrt(w_squared(2));
    w2_store = [w2_store w2];
    w3 = sqrt(w_squared(3));
    w3_store = [w3_store w3];
    w4 = sqrt(w_squared(4));
    w4_store = [w4_store w4];

end