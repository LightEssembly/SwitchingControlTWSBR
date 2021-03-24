function [x_data, u_data] = sbr_regulator_pid_xy(time, x_initial, K_val, Kp1_val, Ki1_val, Kd1_val, N1_val, N_bar, r_tha_val, K2_val, Kp2_val, Ki2_val, Kd2_val, N2_val, N_bar2, r_sig_val, u_max)

    sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

    % Assign values for the controller
    global A B C K Kp1 Ki1 Kd1 N1
    A = [-112.479885835723,-13.7455338822368,4.49919543342893;
        0,0,1;
        1198.72490645226,211.236513337316,-47.9489962580903];
    B = [10.5666559417746;0;-112.611366567498];
    B = [B/2 B/2];
    C = [0 1 0; 0 0 1];
    K = K_val;
    Kp1 = Kp1_val;
    Ki1 = Ki1_val;
    Kd1 = Kd1_val;
    N1 = N1_val;
    
    global A2 B2 C2 K2 Kp2 Ki2 Kd2 N2
    A2 = zeros(2,2); A2(1,2) = 1;
    B2 = zeros(2,2);
    B2(2, 1:2) = [-789.735924265072, 789.735924265072];
    C2 = [1 0;0 1];
    K2 = K2_val;
    Kp2 = Kp2_val;
    Ki2 = Ki2_val;
    Kd2 = Kd2_val;
    N2 = N2_val;
    
    global u r u2 r_sig
    r = [0;0];
    r_sig = [0;0];
    u = [0; 0];
    u2 = [0; 0];
    u_combined = [0; 0];
    
    x = x_initial(1:3)';
    sig = x_initial(4:5)';
    
    terminate_simulation = false;
    dt = 0.004;

    % Simulation started -> working with the robot
    if (clientID>-1)
        % enable the synchronous mode on the client:
        sim.simxSynchronous(clientID,true);
        sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);

        % Prepare signal streams
        sim.simxClearStringSignal(clientID, 'input', sim.simx_opmode_oneshot);
        sim.simxClearStringSignal(clientID, 'controller', sim.simx_opmode_oneshot);
        
        % Setup initial condition for x
        [code, body] = sim.simxGetObjectHandle(clientID, 'SBR', sim.simx_opmode_blocking);
        if code == sim.simx_return_ok
            ang = x_initial(2);
            sim.simxSetObjectOrientation(clientID, body, -1, [0 x_initial(2) x_initial(4)], sim.simx_opmode_blocking);
            code = sim.simxSetObjectPosition(clientID, body, -1, [0 0 cos(ang)*.08+0.02], sim.simx_opmode_blocking);
            if code ~= sim.simx_return_ok
                terminate_simulation = true;
            end
        else
            terminate_simulation = true;
        end
        
        
        % Trigger one time step of simulation
        sim.simxSynchronousTrigger(clientID);

        % Setup variables
        [~,val] = sim.simxGetStringSignal(clientID,'controller',sim.simx_opmode_streaming);
        [~, bodyPos] = sim.simxGetObjectPosition(clientID, body, -1, sim.simx_opmode_streaming);
        lastTime = 0;
        
        x_data = zeros(size(time,2), 7);
        x_data(1, 1:3) = x(1:3);
        x_data(1, 4:5) = sig(1:2);
        u_data = zeros(size(time,2), 6);

        for i=1:length(time)
            if terminate_simulation == true
                break;
            end
            
            [code,val] = sim.simxGetStringSignal(clientID,'controller',sim.simx_opmode_buffer);
            if (code == sim.simx_return_ok && i > 1)
                
                r(1:2) = r_tha_val(1:2,i)*N_bar/2.25;
                r_sig(1:2) = [r_sig_val(1,i)*N_bar2; -r_sig_val(2,i)*N_bar2]/2;
   
                fVal = sim.simxUnpackFloats(val);
                currentTime = fVal(4);
                dt = currentTime-lastTime;
                if dt == 0
                    continue
                end

                x(1:3) = fVal(1:3);
                sig(1:2) = fVal(5:6);
                % Check if the theta is greater than 72 deg -> Unrecoverable
                if abs(x(2)) > 70*pi/180
                    break;
                end
                
                % This computes u u2
                [~] = ode45(@pidFunctionTha, double([time(i) time(i)+dt]), x);
                [~] = ode45(@pidFunctionSig, double([time(i) time(i)+dt]), sig);
                
                %u = r - K*yOut(end, :)';
                lastTime = currentTime;
                u_combined = u + u2;
                u_combined = adjustU(u_combined, u_max);
            end
            [~, bodyPos] = sim.simxGetObjectPosition(clientID, body, -1, sim.simx_opmode_buffer);
            x_data(i, 1:3) = x(1:3);
            x_data(i, 4:5) = sig(1:2);
            x_data(i, 6:7) = bodyPos(1:2);
            u_data(i, 1:2) = u;
            u_data(i, 3:4) = u2;
            u_data(i, 5:6) = u_combined;

            sim.simxClearStringSignal(clientID, 'input', sim.simx_opmode_oneshot);
            sim.simxSetStringSignal(clientID, 'input', sim.simxPackFloats(u_combined'), sim.simx_opmode_oneshot);
            sim.simxSynchronousTrigger(clientID);
            ping = sim.simxGetPingTime(clientID);
            
            % Check if the robot is close to the end of space
            if bodyPos(1) > 15
                break;
            end
        end
        % stop the simulation:
        sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end

    sim.delete(); % call the destructor!
    disp('Program ended');
end

function dxdt = pidFunctionTha(t, x)
    global A B K Kp1 Ki1 Kd1 N1 r u
    persistent t_prev ep_old ep ed ei ed_integ
    dxdt = zeros(3,1);
    
    if isempty(ep_old)
        t_prev = 0;
        ep_old = r - K*x;
        ep = r - K*x;
        ed = 0;
        ei = 0;
        ed_integ = 0;
    end
    dt = t-t_prev;
    
    if dt > 0
        ed_integ = ed_integ + N1*(ep*Kd1 - ed_integ)*dt;
        ep = r-K*x;                 %Calculating P
        %ed = (ep - ep_old)/dt;     %Calculating D
        ei = ei + ep*dt;           %Calculating I
        
        ep_old = ep;
    end
    % Calculating u
    %u = Kp1*ep + (Kd1*ed-ed_integ) + Ki1*ei;
    u = Kp1*ep + N1*(ep*Kd1 - ed_integ) + Ki1*ei;
    % State update
    dxdt(:) = A*x + B*u;
    %u_list = [u_list u];
    t_prev = t;
end
function dxdt = pidFunctionSig(t, x)
    global A2 B2 K2 Kp2 Ki2 Kd2 N2 r_sig u2
    persistent t_prev ep_old ep ed ei ed_integ
    dxdt = zeros(2,1);
    
    if isempty(ep_old)
        t_prev = 0;
        ep_old = r_sig - K2*x;
        ep = r_sig - K2*x;
        ed = 0;
        ei = 0;
        ed_integ = 0;
    end
    dt = t-t_prev;
    
    if dt > 0
        ed_integ = ed_integ + N2*(ep*Kd2 - ed_integ)*dt;
        ep = r_sig-K2*x;                 %Calculating P
        %ed = (ep - ep_old)/dt;     %Calculating D
        ei = ei + ep*dt;           %Calculating I
        
        ep_old = ep;
    end
    % Calculating u
    %u = Kp2*ep + (Kd2*ed-ed_integ) + Ki2*ei;
    u2 = Kp2*ep + N2*(ep*Kd2 - ed_integ) + Ki2*ei;
    % State update
    dxdt(:) = A2*x + B2*u2;
    t_prev = t;
end
function u_val = adjustU(u_val, u_max)
   if u_val(1) > u_max
       u_val(1) = u_max;
   elseif u_val(1) < -1*u_max
       u_val(1) = -1*u_max;
   end
   
   if u_val(2) > u_max
       u_val(2) = u_max;
   elseif u_val(2) < -1*u_max
       u_val(2) = -1*u_max;
   end  
end