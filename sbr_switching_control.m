function [x_data, u_data] = sbr_switching_control(time, pid_vals, sliding_vals, x_initial, r_val, u_maxVal, switchAngle)

    sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);
    
    % PID VALUES
    global A B_pid K Kp Ki Kd N u
    A = 1.0e+03*[-0.112479885835723, -0.013745533882237, 0.004499195433429;
                       0, 0, 0.001;
                   1.198724906452258, 0.211236513337316, -0.047948996258090;];
    B_pid = 1.0e+02*[0.105666559417746; 0; -1.126113665674981];
    K = pid_vals(1:3);
    Kp = pid_vals(4);
    Ki = pid_vals(5);
    Kd = pid_vals(6);
    N = pid_vals(7);
    
    % SLIDING MODE VALUES
    global C epsil sigmoidV tau
    sigmoidV = 0.6825;
    C = sliding_vals(1:3);
    epsil = sliding_vals(4);
    B_smc = [2.369207772795217; 0; -0.094768310911809];
    tau = 1.4e-4;
    
    x = [x_initial'; 0]; 
    r = r_val;
    dt = double(0.004);
    u = 0;
    u_max = u_maxVal;
    
    terminate_simulation = false;

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
            sim.simxSetObjectOrientation(clientID, body, -1, x_initial, sim.simx_opmode_blocking);
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
        
        x_data = zeros(size(time,2), 3);
        x_data(1, 1:3) = x(1:3);
        u_data = zeros(size(time));

        for i=1:length(time)
            if terminate_simulation == true
                break;
            end
            
            [code,val] = sim.simxGetStringSignal(clientID,'controller',sim.simx_opmode_buffer);
            if (code == sim.simx_return_ok && i > 1)
                     
                fVal = sim.simxUnpackFloats(val);
                currentTime = fVal(4);
                dt = currentTime-lastTime;
                
                if dt == 0
                    continue
                end

                x(1:3) = fVal(1:3);
                % Check if the theta is greater than 85 deg -> Unrecoverable
                if abs(x(2)) > 85*pi/180
                    break;
                end
                
                if abs(x(2)) < switchAngle
                    % PID is active
                    [~, yOut] = ode45(@pidFunction, double([time(i) time(i)+dt]), x(1:3));
                else
                    % SMC is active
                    [~, yOut] = ode45(@odeSolv, double([time(i) time(i)+dt]), x);
                    xd = yOut(end, 1); theta = yOut(end, 2); thetad = yOut(end, 3); z = yOut(end, 4);
                    MF = invM_F(xd, theta, thetad);
                    cmu = epsil*z - C*MF;
                    u = cmu/(C*invM(xd, theta, thetad)*B_smc);
                end
                
                if u > u_max
                    u = u_max;
                end
                lastTime = currentTime;
            end
            x_data(i, 1:3) = x(1:3);
            u_data(i) = u;
            sim.simxClearStringSignal(clientID, 'input', sim.simx_opmode_oneshot);
            sim.simxSetStringSignal(clientID, 'input', sim.simxPackFloats(u), sim.simx_opmode_oneshot);
            sim.simxSynchronousTrigger(clientID);
            sim.simxGetPingTime(clientID);
            
            % Check if the robot is close to the end of space
            [~, bodyPos] = sim.simxGetObjectPosition(clientID, body, -1, sim.simx_opmode_buffer);
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
function dxdt = pidFunction(t, x)
    global A B_pid K r Kp Ki Kd N u
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
        ed_integ = ed_integ + N*(ep*Kd - ed_integ)*dt;
        ep = r-K*x;                 %Calculating P
        %ed = (ep - ep_old)/dt;     %Calculating D
        ei = ei + ep*dt;           %Calculating I
        
        ep_old = ep;
    end
    % Calculating u
    %u = Kp*ep + (Kd*ed-ed_integ) + Ki*ei;
    u = Kp*ep + N*(ep*Kd - ed_integ) + Ki*ei;

    
    dxdt(:) = A*x + B_pid*u;
    t_prev = t;
end
function dxdt = odeSolv(t, x)
    global C epsil tau
    dxdt = zeros(4,1);
    
    xd = x(1);
    theta = x(2);
    thetad = x(3);
    z = x(4);

    MF = invM_F(xd, theta, thetad);
    sigma = C*[xd, theta, thetad]';
    cmu = epsil*z - C*MF;
    MU = pinv(C)*cmu;
    
    dxdt(1:3) = MF+MU;
    dxdt(4) = (sigmoid(sigma) - z)*(1/tau);
end
function y = invM_F(xd, theta, thetad)
    y = zeros(3,1);

    y(1) = (119*cos(theta)*((36589*xd)/5000 - (36589*thetad)/125000 + (116739*sin(theta))/250000))/(1562500*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000)) - (1327*((5624*thetad)/5575 - (5624*xd)/223))/(156250000*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
    y(2) = theta;
    y(3) = (119*cos(theta)*((5624*thetad)/5575 - (5624*xd)/223))/(1562500*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000)) - (3373448322479634243553*((36589*xd)/5000 - (36589*thetad)/125000 + (116739*sin(theta))/250000))/(2882303761517117440000000*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
end
function y = sigmoid(x)
    global sigmoidV
    y = x / (abs(x) + sigmoidV);
end
function y = inv_CinvM(cmu, xd, theta, thetad)
    C_invM = zeros(1,3);
    
    C_invM(1) = (119*cos(theta))/(1562500*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000)) - 1327/(156250000*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
    C_invM(2) = 1;
    C_invM(3) = (119*cos(theta))/(1562500*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000)) - 3373448322479634243553/(2882303761517117440000000*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
    
    y = C_invM\cmu;
end
function y = invM(xd, theta, thetad)
    y = zeros(3,3);

    y(1,1) = -1327/(156250000*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
    y(1,2) = 0;
    y(1,3) = (119*cos(theta))/(1562500*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
    
    y(2,1) = 0;
    y(2,2) = 1;
    y(2,3) = 0;

    y(3,1) = (119*cos(theta))/(1562500*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
    y(3,2) = 0;
    y(3,3) = -3373448322479634243553/(2882303761517117440000000*((14161*cos(theta)^2)/3906250000 - 4476565923930474641194831/720575940379279360000000000000));
end
