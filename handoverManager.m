classdef handoverManager < handle
    properties
        % Network entities
        gNBs
        UE
        networkSimulator
        numCells

        % Configuration parameters
        scanPeriod = 0.01          % Periodic scan interval for handover decision (every 0.01s). For a 10s simulation = 1000 decision steps.
        scanStartTime = 0.0795     % Initial scan time: gNBs start receiving SRS around 0.0715s; by 0.0795s, 4 updates are available for averaging
        hysteresis = 1             % Hysteresis margin (dB) for A3 event
        sinrThreshold = 20         % SINR threshold (dB) for A5 event

        % SINR measurement storage
        ulSINR                    % Uplink SINR matrix [numCells × time]
        gNBCount                 % Count of SINR measurements per gNB
        ulSINRAverage            % Averaged SINR from last 4 measurements [numCells × 1]
        ullastSINR
        connectedulSINR = []     % SINR measurements from currently connected gNB

        % Handover strategy pool
        handoverPolicy = {}       % Cell array storing different handover models

        % DQN agent logging
        dqnAgent_data            % Struct for reinforcement learning agent data
        handover_num = 0         % Total number of handovers performed
        step = 1                 % Current decision step counter

        % UCB-based SMART-S algorithm parameters
        SINR_Threshold = 23      % SINR threshold for A2 event (dB)
        Hyst = 2                 % Hysteresis value for A2 event (dB)
        ell = 0.6                % Exploration factor in UCB reward function
        tau = 0.08               % Allowable delay before A2-triggered handover (in seconds)
        taustep = 3              % Column index for tau in SINR matrix (manually set)
        connectionStartIdx = 1   % Start index for current connection duration
        UCBdata                  % Struct storing SMART-S related data
        TransmissionRate_Threshold = 93e6  % Throughput threshold in bps
        TransmissionRate_Hyst = 1e6        % Throughput hysteresis offset, 1 Mbps

        % DDPG agent related data
        ddpg_data
        TTT = 10                 % Time-To-Trigger
        timer = 0               % Trigger countdown timer
        prev_trigger_step = 0   % Step when last handover was triggered

        % Mode: 'eval' or 'train'
        mode = 'eval'

        % Throughput tracking
        lastTxBytes = []        % Previously transmitted bytes
        lastThroughput = 0      % Last calculated throughput in Mbps
        allthroughput = []      % History of throughput values
        allthroughput_time = [] % Timestamps for throughput measurements
        stepsSinceLastHO = 0    % Steps since the last handover
        lastTxTime = 0          % Last timestamp of throughput measurement

        ulApp                   % Uplink application handle
        dlApp                   % Downlink application handle
    end

    methods
        %% Constructor: Initialize with network entities
        function obj = handoverManager(UE,gNBs, networkSimulator, ulApp, dlApp, varargin) 
            obj.gNBs = gNBs;
            obj.UE = UE;
            obj.networkSimulator = networkSimulator;
            obj.numCells = length(gNBs);

            obj.ulApp = ulApp;
            obj.dlApp = dlApp;

            obj.ulSINR = [];
            obj.ulSINRAverage = [];
            obj.gNBCount = zeros(obj.numCells, 1); 

            % Initialize DQN logging structure
            obj.dqnAgent_data = struct( ...
                'state', [], ...
                'reward', [], ...
                'action', [], ...
                'next_state', [], ...
                'total_reward', 0, ...
                'done', false);

            % Initialize SMART-S UCB logging structure
            % R_hat: estimated reward
            % T: selection count for each gNB
            % H: total handovers per UE
            obj.UCBdata = struct( ...
                'R_hat', zeros(length(UE), obj.numCells), ...
                'T', zeros(length(UE), obj.numCells), ...
                'H', zeros(length(UE), 1));

            % Initialize DDPG logging structure
            obj.ddpg_data = struct( ...
                'state', [], ...
                'reward', [], ...
                'action', [], ...
                'next_state', [], ...
                'total_reward', 0, ...
                'done', false);

            % Set operational mode
            if nargin > 3
                if strcmp(varargin{1}, 'train') || strcmp(varargin{1}, 'eval')
                    obj.mode = varargin{1};
                else
                    error('Invalid mode specified. Mode should be either "train" or "eval".');
                end
            end

            % Register default handover models (A3, DQN, UCB, DDPG)
            obj.addHandoverModel(@obj.a3Condition); 
            obj.addHandoverModel(@obj.DQNPolicy); 
            obj.addHandoverModel(@obj.UCBPolicy);
            obj.addHandoverModel(@obj.DDPGPolicy);
            
            % Register event listener and periodic execution
            addlistener(gNBs, 'PacketReceptionEnded', @(src, eventData) obj.handlePacketReception(src, eventData));
            scheduleAction(networkSimulator, @obj.checkHandoverPolicy, [], obj.scanStartTime, obj.scanPeriod);
        end

        %% Register a new handover model to the policy pool
        function addHandoverModel(obj, modelHandle)
            obj.handoverPolicy{end + 1} = modelHandle;
        end

        %% 1. Handle packet reception (data collection)
        function handlePacketReception(obj, src, event)
            if strcmp(event.EventName, "PacketReceptionEnded") && ...
               strcmp(event.Data.SignalType, "SRS") && ...
               event.Data.CurrentTime * 1e3 > 70

                if event.Data.RNTI == obj.UE.ID - 5
                    index = src.ID;
                    obj.gNBCount(index) = obj.gNBCount(index) + 1;
                    obj.ulSINR(index, obj.gNBCount(index)) = event.Data.ChannelMeasurements.SINR;
                end
            end
        end

        %% Collect uplink throughput from log
        function throughputMbps = getULThroughput(obj)
            logPath = 'D:\goproject\src\mp-quic\mp-quic-conext17-1\example\reqres_file_loop\client\throughput.log';
            throughputMbps = 0;
            if ~isfile(logPath)
                return;
            end

            try
                fid = fopen(logPath, 'r');
                fseek(fid, -2048, 'eof');
                data = textscan(fid, '%s', 'Delimiter', '\n');
                fclose(fid);
                lines = data{1};
                lastLine = lines{end};

                tokens = regexp(lastLine, ' ', 'split');
                currentTxBytes = str2double(tokens{14});
                currentTime = obj.networkSimulator.CurrentTime;

                if isempty(obj.lastTxBytes)  % Initialization
                    obj.lastTxBytes = currentTxBytes;
                    obj.lastTxTime = currentTime;
                    return;
                end

                if obj.lastThroughput > currentTxBytes
                    return;
                end

                if currentTxBytes == obj.lastTxBytes
                    throughputMbps = obj.lastThroughput;
                    return;
                end

                deltaBits = (currentTxBytes - obj.lastTxBytes) * 8;
                deltaTime = currentTime - obj.lastTxTime;  
                throughputMbps = deltaBits / (deltaTime * 1e6);

                fprintf('Step %d | deltaTime = %.4f s | deltaBits = %.0f bits | Throughput = %.3f Mbps\n', ...
                    obj.step, deltaTime, deltaBits, throughputMbps);    

                obj.lastTxBytes = currentTxBytes;
                obj.lastTxTime = currentTime;
                obj.lastThroughput = throughputMbps;
                obj.allthroughput = [obj.allthroughput, throughputMbps];
                obj.allthroughput_time = [obj.allthroughput_time, currentTime];

            catch ME
                throughputMbps = 0;
                warning(ME.identifier, "Log read error: %s", ME.message);
            end  
        end

      %% Evaluate handover policy
        function checkHandoverPolicy(obj, ~, ~)
            % Calculate average uplink SINR over recent 4 time steps
            obj.ulSINRAverage = mean(obj.ulSINR(:, end-3:end), 2);
            obj.ullastSINR = obj.ulSINR(:, end);
        
            % Record SINR of currently connected gNB
            currentIdx = find([obj.gNBs.ID] == obj.UE.GNBNodeID, 1);
            obj.connectedulSINR = [obj.connectedulSINR, obj.ulSINR(currentIdx, end-3:end)];
        
            % Use selected handover policy 
            model = obj.handoverPolicy{1};
            targetGNB = model(obj.ulSINRAverage, obj.UE, obj.gNBs);
        
            % Execute handover if target gNB is valid
            if ~isempty(targetGNB)
                obj.executeHandover(targetGNB);
                obj.handover_num = obj.handover_num + 1;
            end
        end

        %% 1、A3
        function targetGNB = a3Condition(obj, currentSINR, UE, gNBs)
            currentIdx = find([gNBs.ID] == UE.GNBNodeID, 1);
            S_SINR = currentSINR(currentIdx);
          

            for i = 1:length(gNBs)
                if i == currentIdx, continue; end
                if currentSINR(i) > (S_SINR + obj.hysteresis)
                    targetGNB = gNBs(i);
                    return;
                end
            end
            targetGNB = [];
        end

        %% 2、DQN-based handover strategy
        %% DQN-based handover strategy
        function targetGNB = DQNPolicy(obj, currentSINR, UE, gNBs)
            if strcmp(obj.mode, 'train')
                % Index of currently connected gNB
                currentIdx = find([gNBs.ID] == UE.GNBNodeID, 1);
        
                % Build state: normalized SINR, connection status, delta to best SINR,
                % normalized throughput, time since last handover, handover count
                sinrMax = 30; sinrMin = 0;
                normSINR = min(max((currentSINR - sinrMin) / (sinrMax - sinrMin), 0), 1);
                oneHot = zeros(length(gNBs), 1);
                oneHot(currentIdx) = 1;
                [~, bestIdx] = max(normSINR);
                deltaToBest = normSINR(bestIdx) - normSINR(currentIdx);
                tp_norm = min(obj.lastThroughput / 1.0, 1);
                timeSinceLastHO = min(obj.stepsSinceLastHO / 100, 1);
                handoverCount = min(obj.handover_num / 100, 1);
                state = [normSINR; oneHot; deltaToBest; tp_norm; timeSinceLastHO; handoverCount];
        
                % Select action from policy
                action = py.DDQN.get_action(state);
                actionmat = int32(action) + 1;
                targetGNB = gNBs(actionmat);
        
                % Handover decision
                isHandover = (targetGNB.ID ~= UE.GNBNodeID);
                if ~isHandover
                    targetGNB = [];
                    obj.stepsSinceLastHO = obj.stepsSinceLastHO + 1;
                else
                    obj.stepsSinceLastHO = 0;
                end
        
                % Calculate reward
                delta_sinr_dB = currentSINR(actionmat) - currentSINR(currentIdx);
                reward = 6 * normSINR(currentIdx) + 2 * tp_norm;
                if isHandover
                    reward = reward - 4;
                    if delta_sinr_dB < 3
                        reward = reward - 3;  % penalty for unnecessary handover
                    end
                else
                    if currentSINR(currentIdx) < 25 && obj.stepsSinceLastHO > 2
                        reward = reward - 4;  % penalty for sticking to poor connection
                    end
                end
                reward = reward - 0.05 * obj.handover_num;
        
                % Store experience
                obj.dqnAgent_data.state(:, obj.step) = state;
                obj.dqnAgent_data.action(obj.step) = action;
                obj.dqnAgent_data.reward(obj.step) = reward;
                obj.dqnAgent_data.total_reward = obj.dqnAgent_data.total_reward + reward;
        
                % Perform learning
                if obj.step >= 2
                    learn_success = py.DDQN.learn( ...
                        obj.dqnAgent_data.state(:, obj.step - 1), ...
                        int32(obj.dqnAgent_data.action(obj.step - 1)), ...
                        double(obj.dqnAgent_data.reward(obj.step - 1)), ...
                        obj.dqnAgent_data.state(:, obj.step), ...
                        logical(obj.dqnAgent_data.done));
                    if ~learn_success
                        error('DQN learning failed');
                    end
                end
        
                % Print step info
                fprintf('Step %d: Action = %d, Reward = %.3f\n', ...
                    obj.step, action, reward);
        
                obj.step = obj.step + 1;
                global last_handover_num;
                last_handover_num = obj.handover_num;
        
                % Termination condition
                if obj.step >= 160 || obj.networkSimulator.CurrentTime >= 2
                    obj.dqnAgent_data.done = true;
                else
                    obj.dqnAgent_data.done = false;
                end
        
            elseif strcmp(obj.mode, 'eval')
                % === Evaluation mode ===
                currentIdx = find([gNBs.ID] == UE.GNBNodeID, 1);
        
                % Build observation
                sinrMax = 30; sinrMin = 0;
                normSINR = min(max((currentSINR - sinrMin) / (sinrMax - sinrMin), 0), 1);
                oneHot = zeros(length(gNBs), 1);
                oneHot(currentIdx) = 1;
                [~, bestIdx] = max(normSINR);
                deltaToBest = normSINR(bestIdx) - normSINR(currentIdx);
                tp_norm = min(obj.lastThroughput / 1.0, 1);
                timeSinceLastHO = min(obj.stepsSinceLastHO / 100, 1);
                handoverCount = min(obj.handover_num / 100, 1);
                obsInfo = [normSINR; oneHot; deltaToBest; tp_norm; timeSinceLastHO; handoverCount];
        
                % Load pretrained model and infer (greedy policy)
                state_dim = int32(14); action_dim = int32(5);
                config = py.dict(pyargs( ...
                    'learning_rate', 0.0005, ...
                    'gamma', 0.98, ...
                    'epsilon', 1.0, ...
                    'epsilon_decay', 0.95, ...
                    'epsilon_min', 0.01, ...
                    'batch_size', int32(128), ...
                    'memory_size', int32(50000), ...
                    'target_update_freq', int32(100), ...
                    'use_dueling', true, ...
                    'n_step', int32(2) ...
                ));
                py.DDQN.init_agent(state_dim, action_dim, config);
                py.DDQN.load_model('trained_model_17.pth');
        
                % Inference
                action = py.DDQN.get_action(obsInfo);
                actionmat = int32(action) + 1;
                targetGNB = gNBs(actionmat);
        
                % Handover decision
                isHandover = (targetGNB.ID ~= UE.GNBNodeID);
                if ~isHandover
                    targetGNB = [];
                    obj.stepsSinceLastHO = obj.stepsSinceLastHO + 1;
                else
                    obj.stepsSinceLastHO = 0;
                end
            end
        end






        %% 3. UCB-based handover policy
   
        function targetGNB = UCBPolicy(obj, currentSINR, UE, gNBs)
            % init
            targetGNB = [];
            currentIdx = find([gNBs.ID] == UE.GNBNodeID, 1);
        
            if obj.networkSimulator.CurrentTime > obj.tau
                % Check if SINR has stayed below threshold for tau duration
                if all(obj.ulSINR(currentIdx, end - obj.taustep + 1 : end) < obj.SINR_Threshold)
        
                    % Estimate transmission rate using Shannon capacity
                    transmissionRate = obj.calculateTransmissionRate(currentSINR, gNBs);
        
                    % Identify candidate base stations
                    acceptable_base_stations = find((transmissionRate(:, 1) >= ...
                        obj.TransmissionRate_Threshold + obj.TransmissionRate_Hyst) & ...
                        ((1:length(gNBs))' ~= currentIdx));
        
                    if ~isempty(acceptable_base_stations)
                        % Compute UCB scores
                        ucb_scores = zeros(size(acceptable_base_stations));
                        for i = 1:length(acceptable_base_stations)
                            bs = acceptable_base_stations(i);
                            if obj.UCBdata.T(1, bs) == 0
                                ucb_scores(i) = Inf;
                            else
                                ucb_scores(i) = obj.UCBdata.R_hat(1, bs) + ...
                                    obj.ell * sqrt(2 * log(obj.UCBdata.H(1)) / obj.UCBdata.T(1, bs));
                            end
                        end
        
                        % Select base station with highest UCB score
                        [~, index] = max(ucb_scores);
                        new_base_station = acceptable_base_stations(index);
        
                        % Calculate cumulative throughput of current connection
                        if obj.connectionStartIdx <= length(obj.lastThroughput)
                            currentThroughput = sum(obj.lastThroughput(obj.connectionStartIdx:end));
                        else
                            currentThroughput = 0;
                        end
        
                        % Update connection start index
                        if isempty(obj.lastThroughput)
                            obj.connectionStartIdx = 1;
                        else
                            obj.connectionStartIdx = size(obj.lastThroughput, 2) + 1;
                        end
        
                        % Update UCB statistics
                        obj.UCBdata.H(1) = obj.UCBdata.H(1) + 1;
                        obj.UCBdata.T(1, new_base_station) = obj.UCBdata.T(1, new_base_station) + 1;
                        obj.UCBdata.R_hat(1, new_base_station) = ...
                            (obj.UCBdata.R_hat(1, new_base_station) * ...
                            (obj.UCBdata.T(1, new_base_station) - 1) + currentThroughput) / ...
                            obj.UCBdata.T(1, new_base_station);
        
                        % Final handover decision
                        targetGNB = gNBs(new_base_station);
                        obj.handover_num = obj.handover_num + 1;
                    end
                end
            end
        
            obj.step = obj.step + 1;
        end
        
        % Compute transmission rate from SINR using Shannon capacity
        function rate = calculateTransmissionRate(~, currentSINR, gNBs)
            rate = zeros(size(currentSINR));
            for i = 1:length(gNBs)
                bandwidth = gNBs(i).ChannelBandwidth;
                rate(i) = bandwidth * log2(1 + currentSINR(i));
            end
        end


        %% 4.DDPG-based handover strategy
        function targetGNB = DDPGPolicy(obj, currentSINR, UE, gNBs)
            if strcmp(obj.mode, 'train')
                % 1. Get current gNB index
                currentIdx = find([gNBs.ID] == UE.GNBNodeID, 1);
                if isempty(currentIdx)
                    targetGNB = [];
                    return;
                end
        
                % 2. Construct normalized state vector
                sinrMax = 30; sinrMin = 0;
                normSINR = (currentSINR - sinrMin) / (sinrMax - sinrMin);
                normSINR = min(max(normSINR, 0), 1);
        
                ulThroughput = obj.lastThroughput * 1000;  % Mbps → kbps
                throughput_norm = min(max(ulThroughput / 10000, 0), 1);
        
                state = normSINR;
        
                % 3. Query action from DDPG agent
                HOM = py.DDPG.get_action(state);
        
                % 4. Evaluate candidate gNBs
                bs_target = 0;
                best_sinr = currentSINR(currentIdx) + HOM;
                for i = 1:length(currentSINR)
                    if i ~= currentIdx && currentSINR(i) > best_sinr
                        best_sinr = currentSINR(i);
                        bs_target = i;
                    end
                end
        
                % 5. Apply TTT mechanism
                if bs_target > 0
                    if obj.timer < obj.TTT - 1
                        obj.timer = obj.timer + 1;
                        bs_target = currentIdx;
                    else
                        obj.timer = 0;
                    end
                else
                    if obj.timer < obj.TTT
                        obj.timer = obj.timer + 1;
                    else
                        obj.timer = 0;
                    end
                end 
        
                isHandover = (currentIdx ~= bs_target) && (bs_target > 0);
                if isHandover
                    targetGNB = gNBs(bs_target);
                else
                    targetGNB = [];
                    bs_target = currentIdx;
                end
        
                % 6. Compute reward
                w1 = 0.1; w2 = 0.1;
                reward = throughput_norm;
                if isHandover
                    reward = reward - w1;
                    if obj.step - obj.prev_trigger_step < 5
                        reward = reward - w2;  % penalize ping-pong handover
                    end
                    obj.prev_trigger_step = obj.step;
                end
        
                % 7. Store transition
                obj.ddpg_data.state(:, obj.step) = state;
                obj.ddpg_data.action(obj.step) = bs_target;
                obj.ddpg_data.reward(obj.step) = reward;
                obj.ddpg_data.total_reward = obj.ddpg_data.total_reward + reward;
        
                % 8. Perform DDPG learning
                if obj.step >= 2
                    learn_success = py.DDPG.learn( ...
                        obj.ddpg_data.state(:, obj.step - 1), ...
                        int32(obj.ddpg_data.action(obj.step - 1)), ...
                        double(obj.ddpg_data.reward(obj.step - 1)), ...
                        obj.ddpg_data.state(:, obj.step), ...
                        logical(obj.ddpg_data.done));
                    if ~learn_success
                        error('DDPG learning failed');
                    end
                end
        
                fprintf('Step %d: Action = %d, Reward = %.3f\n', obj.step, bs_target, reward);
                obj.step = obj.step + 1;
                global last_handover_num;
                last_handover_num = obj.handover_num;
        
            elseif strcmp(obj.mode, 'eval')
                % Load trained DDPG model for inference
                state_dim = 5;
                action_dim = 1;
                actor_path = 'trained_ddpg_actor_model3.pth';
                critic_path = 'trained_ddpg_critic_model3.pth';
                py.DDPG.init_agent(int32(state_dim), int32(action_dim));
                py.DDPG.load_model(actor_path, critic_path);
        
                currentIdx = find([gNBs.ID] == UE.GNBNodeID, 1);
                if isempty(currentIdx)
                    targetGNB = [];
                    return;
                end
        
                sinrMax = 30; sinrMin = 0;
                normSINR = (currentSINR - sinrMin) / (sinrMax - sinrMin);
                normSINR = min(max(normSINR, 0), 1);
                obsInfo = normSINR;
        
                % Get action (HOM) from actor
                HOM = py.DDPG.get_action(obsInfo, false);
        
                bs_target = 0;
                best_sinr = currentSINR(currentIdx) + HOM;
                for i = 1:length(currentSINR)
                    if i ~= currentIdx && currentSINR(i) > best_sinr
                        best_sinr = currentSINR(i);
                        bs_target = i;
                    end
                end
        
                % Apply TTT
                if bs_target > 0
                    if obj.timer < obj.TTT - 1
                        obj.timer = obj.timer + 1;
                        bs_target = currentIdx;
                    else
                        obj.prev_trigger_step = obj.step;
                        obj.timer = 0;
                    end
                else
                    if obj.timer < obj.TTT
                        obj.timer = obj.timer + 1;
                    else
                        obj.timer = 0;
                    end
                end
        
                isHandover = (currentIdx ~= bs_target) && (bs_target > 0);
                if isHandover
                    targetGNB = gNBs(bs_target);
                else
                    targetGNB = [];
                end
            end
        end



        %% Emergency handover
        function emergencyHandover(obj)
            currentIdx = find([obj.gNBs.ID] == obj.UE.GNBNodeID, 1);

            for i = 1:length(obj.ulSINRAverage)
                if i ~= currentIdx && obj.ulSINRAverage(i) > obj.sinrThreshold
                    targetGNB = obj.gNBs(i);
                    obj.executeHandover(targetGNB);
                    return;
                end
            end
        end

        %% Execute handover
        function executeHandover(obj, targetGNB)
            currentGNB = obj.gNBs([obj.gNBs.ID] == obj.UE.GNBNodeID);

            disconnect(currentGNB, obj.UE);
            reconnectUE(targetGNB, obj.UE);

            obj.resetTraffic(targetGNB);

            disp("Handover executed: UE" + obj.UE.ID + " from gNB" + ...
                currentGNB.ID + " to gNB" + targetGNB.ID);
        end

        %% Reset traffic flow after handover
        function resetTraffic(obj, targetGNB)
            addTrafficSource(targetGNB, obj.dlApp, 'DestinationNode', obj.UE); 
            addTrafficSource(obj.UE, obj.ulApp); 
        end
    end
end
