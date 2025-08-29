%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: main_training_script.m
%
% Description:
%   This MATLAB script integrates a 5G wireless network simulation with a
%   Python-based DDPG\DQN reinforcement learning agent to perform
%   intelligent handover decisions for a UAV equipped with one UE and 
%   five gNBs. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Main control script (training + plotting)
% Initialize Python environment
pe = pyenv('Version', 'D:\anaconda3\envs\MATLABProj\python.exe');
if count(py.sys.path, pwd) == 0
    insert(py.sys.path, int32(0), pwd);
end
%========DDQN============
py.importlib.import_module('DDQN');
%Load Python model for training continuation
%py.DDQN.load_checkpoint('interrupted_python_8.pth');
% % Initialize agent
% State dimension and action dimension
state_dim = int32(14);  
action_dim = int32(5);   

% Build configuration dictionary
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

% Initialize agent
py.DDQN.init_agent(state_dim, action_dim, config);

%=======DDPG=========
% py.importlib.import_module('DDPG');
% 
% state_dim = 5;     % [normSINR]
% action_dim = 1;    % HOM
% 
% % init agent
% init_success = py.DDPG.init_agent(int32(state_dim), int32(action_dim));
%====================

global last_handover_num; 
num_episodes = 50; % number of training episodes
all_rewards = cell(num_episodes, 1); % store rewards per episode
all_handovers = zeros(num_episodes, 1); % store handover counts per episode

%% Main training loop
for ep =1:num_episodes
    fprintf('\n=== Starting training episode %d/%d ===\n', ep, num_episodes);
    
    % Reset agent state (cumulative rewards and records)
    py.DDQN.reset_episode(); 
   
    % Execute single episode training
    train(); 
    
    % Get episode training results
    info = py.DDQN.get_agent_info();
    step_rewards = double(py.list(info{'step_rewards'}));
    epsilon = double(info{'epsilon'});
    disp(epsilon);
    
     % Store data
    all_rewards{ep} = step_rewards;
    all_handovers(ep) = last_handover_num; % record handover count
    
    % Display info in real time
    fprintf('Handover count this episode: %d\n', last_handover_num);
    ep_avg = mean(all_rewards{ep});
    fprintf('Episode %d average reward: %.4f\n', ep, ep_avg);

    figure(1); clf;
    subplot(2,1,1);
    plot(step_rewards); title(sprintf('Rewards (Episode %d)', ep));
    
    subplot(2,1,2);
    bar(last_handover_num); title('Handover Count');
    drawnow;
    
    % Save checkpoint (including handover count)
    save(sprintf('checkpoint_ep%02d.mat', ep),...
         'step_rewards', 'last_handover_num'); % modified saved variables
    if ep==20 % save every 20 episodes
        try
            % Save Python model
            py.DDQN.save_checkpoint('interrupted_python_8.pth');
            
            % Save MATLAB variables
            if exist('ep', 'var') && exist('all_rewards', 'var')
                save('interrupted_matlab_8.mat', ...
                    'all_rewards', 'all_handovers', 'last_handover_num', 'ep', 'num_episodes');
            else
                save('interrupted_matlab.mat', 'num_episodes');
            end
            
            fprintf('Models saved as:\n  - interrupted_python_8.pth\n  - interrupted_matlab_8.mat\n');
        catch ME
            fprintf('Save failed: %s\n', ME.message);
        end
    end
    
   cleanupUDPPorts();
   
end

model_path = fullfile(pwd, 'trained_model_17.pth'); % save in current MATLAB folder
save_success = py.DDQN.save_model(model_path);
if save_success
    fprintf('Model saved at: %s\n', model_path);
else
    error('Model saving failed');
end

%% Post-processing and analysis
figure(2);
plot(all_handovers,'MarkerFaceColor','auto');
title('Handover Count per Episode');

figure(3);
ep_avg = cellfun(@mean, all_rewards);
yyaxis left; plot(ep_avg); ylabel('Average Reward');
yyaxis right; plot(all_handovers); ylabel('Handover Count');
title('Reward and Handover Correlation');

% Save final results
save('training_log_5_18.mat', 'all_rewards', 'all_handovers', 'ep_avg');
disp('Training complete, data saved');



%% network
function train()
    rng("default") % Reset random number generator
    %numFrameSimulation = 10; % Simulation time in terms of number of 10 ms frames
    networkSimulator = wirelessNetworkSimulator.init;
    
   % gNB and UE configuration (1UE+5gNBs)
    gNBPositions = [0 0 25; 350 0 25;700 0 25;180 300 25; 550 300 25 ];      
    % Moving 2.6 GHz band
    gNBNames = "gNB-" + (1:size(gNBPositions,1));
    gNBs = nrGNB(Name=gNBNames,Position=gNBPositions,CarrierFrequency=2.6e9,ChannelBandwidth=20e6,SubcarrierSpacing=30e3,...
        NumTransmitAntennas=16,NumReceiveAntennas=8,ReceiveGain=11,DuplexMode="TDD");
    uePositions =[400 200 150]; %
    UENames = "UE-" + (1:size(uePositions,1));
    UE = nrUE(Name=UENames,Position=uePositions,NumTransmitAntennas=4,NumReceiveAntennas=2,ReceiveGain=11);
    
    configureULforSRS(UE,gNBs);
    rlcBearerConfig = nrRLCBearerConfig(SNFieldLength=6,BucketSizeDuration=10); 
    connectUE(gNBs(2),UE,RLCBearerConfig=rlcBearerConfig);% Initially connect UE to gNB(2)
    % Add nodes and channels
    addNodes(networkSimulator,gNBs);  
    addNodes(networkSimulator,UE)    
    %channel
    channelConfig = struct("DelayProfile", "CDL-C", "DelaySpread", 100e-9);
    channels = createCDLChannels(channelConfig, gNBs, UE);
    customChannelModel = hNRCustomChannelModel(channels,struct(PathlossMethod="nrPathloss"));
    addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);
    
    % Enable mobility
    enableMobility = true;
    if enableMobility
        ueSpeedRange = [1000 1500]; % In meters per second, UAV flying speed is 10-15 m/s (scaled by 100)
        ueWithMobility = UE; 
        % Add random waypoint mobility to the selected UE
        addMobility(ueWithMobility,SpeedRange=ueSpeedRange,BoundaryShape="rectangle",Bounds=[400 200 800 600]) % Center at (400,200), rectangle width and height
    end
    
    %% Traffic Configuration
    
    appDataRate = 1e3; % Application data rate in kilo bits per second (kbps) 
    % UL
    ulApps=networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(UE,ulApps); 
    
    % DL
    
    dlApps = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(gNBs(2),dlApps,DestinationNode=UE);
    
    
    %% Network topology visualization
    networkVisualizer = helperNetworkVisualizer(SampleRate=100); % Sample rate is visualization refresh rate in Hz
    addNodes(networkVisualizer,gNBs);
    addNodes(networkVisualizer,UE);
   
    showBoundaries(networkVisualizer,gNBPositions,200);% Show circular boundaries
    
    %% Mobility management
    handoverManager(UE,gNBs, networkSimulator,ulApps,dlApps,'train');
    addlistener(UE, 'AppDataReceived', @(src, eventData) UEToClient(src, eventData)); 
    addlistener(gNBs, 'AppDataReceived', @(src, eventData) GNBToServer(src, eventData)); 

    %% Run simulation
    triggerFcn(networkSimulator,ulApps,dlApps,UE);

end


%% Event trigger
function triggerFcn(networkSimulator, ulApps, dlApps, UE)
    global simulationTime udpsocket1 udpsocket2;
    simulationTime = 0;
    maxSimulationTime = 2; % maximum simulation time (seconds)

    ip = '10.92.8.105';
    udpsocket1 = udpport("datagram", "LocalHost", ip, "LocalPort", 8000, "Timeout", 100); 
    udpsocket1.OutputDatagramSize = 65507;
    udpsocket2 = udpport("datagram", "LocalHost", ip, "LocalPort", 8004, "Timeout", 100);
    udpsocket2.OutputDatagramSize = 65507;

    configureCallback(udpsocket1, "datagram", 1, @(src, ~) ClientToUE(src, networkSimulator, ulApps, UE));
    configureCallback(udpsocket2, "datagram", 1, @(src, ~) ServerAckToGNB(src, networkSimulator, dlApps, UE.ID));
    disp('start...');
     % Main loop: periodically check simulation time and allow callbacks
    while simulationTime < maxSimulationTime
        drawnow; % refresh event queue, process callbacks
        pause(0.1); % brief pause to reduce CPU usage
    end
    % Clear callbacks and ports
    configureCallback(udpsocket1, "off");
    configureCallback(udpsocket2, "off");
    clear udpsocket1
    clear udpsocket2
    disp('[Trigger] Simulation finished');
end


% When port1 receives data, process the message and assign uplink traffic to UE1 or UE2 based on IP
function ClientToUE(src,networkSimulator,ulApps,UE)
    % Enable forwarding flag of the corresponding gNB
    networkSimulator.Nodes{UE.GNBNodeID,1}.TrafficManager.isForward=true;

    global randomPort1 simulationTime;
    if isempty(simulationTime)
        simulationTime = 0; % Initialize simulationTime as the initial time
    end

    if src.NumDatagramsAvailable>0
        data = read(src,src.NumDatagramsAvailable);
        for i=1:length(data)
            sendData=data(1,i).Data; % row vector
            originalLength=length(sendData); % original length
            randomPort1=data(1,i).SenderPort;
            lengthField = typecast(uint16(originalLength), 'uint8'); % data length field (2 bytes)
            packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)]; % row vector
        
            ulApps.pAppData=double(packetedData'); % ulApp needs a column vector
            
            simulationTime = simulationTime + ulApps.pTransmissionTime;
            run(networkSimulator, simulationTime);

        end

    end

end

% gNBs forwarding
function GNBToServer(~,event)
    global udpsocket2;
    receiveData=event.Data.Packet; % column vector
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16');
    originalData = receiveData(3:2 + originalLength); % extract original data
    write(udpsocket2, originalData', "10.92.8.105", 6121);
end

% port2 receives, ack to gNB
function ServerAckToGNB(src,networkSimulator, dlApps, UEID)
    global udpsocket1;
    if src.NumDatagramsAvailable>0
        data = read(src, src.NumDatagramsAvailable);
        for i=1:length(data)
            originalData = data(1,i).Data; % row vector
            originalLength = length(originalData);
            lengthField = typecast(uint16(originalLength), 'uint8');
            packetedData = [lengthField, originalData, zeros(1, 1500 - originalLength - 2)];
            dlApps.pAppData = double(packetedData'); % column vector
            networkSimulator.Nodes{UEID, 1}.TrafficManager.isForward = true;
            run(networkSimulator, dlApps.pTransmissionTime);
            write(udpsocket1, packetedData', "10.92.8.105", 6120);
        end
    end
end

function cleanupUDPPorts()
    global udpsocket1 udpsocket2;
    if ~isempty(udpsocket1)
        configureCallback(udpsocket1,"off");
        clear udpsocket1;
    end
    if ~isempty(udpsocket2)
        configureCallback(udpsocket2,"off");
        clear udpsocket2;
    end
end




function channels = createCDLChannels(channelConfig,gNBs,UEs)
%createCDLChannels Create channels between gNBs and UEs in a cell
%   CHANNELS = createCDLChannels(CHANNELCONFIG,GNB,UES) creates channels
%   between GNB and UES in a cell.
%
%   CHANNELS is a N-by-N array where N is the number of nodes in the cell.
%
%   CHANNLECONFIG is a struct with these fields - DelayProfile and
%   DelaySpread.
%
%   GNB is an array of nrGNB object.
%
%   UES is an array of nrUE objects.

    numUEs = length(UEs);
    numNodes = length(gNBs) + numUEs;
    % Create channel matrix to hold the channel objects
    channels = cell(numNodes,numNodes);
    for i=1:length(gNBs)
        
        
        % Get the sample rate of waveform
        waveformInfo = nrOFDMInfo(gNBs(i).NumResourceBlocks,gNBs(i).SubcarrierSpacing/1e3);
        sampleRate = waveformInfo.SampleRate;
        
        
        for ueIdx = 1:numUEs
            % Configure the uplink channel model between gNB and UE
            channel = nrCDLChannel;
            channel.DelayProfile = channelConfig.DelayProfile;
            channel.DelaySpread = channelConfig.DelaySpread;
            channel.Seed = 73 + (ueIdx - 1);
            channel.CarrierFrequency = gNBs(i).CarrierFrequency;
            channel = hArrayGeometry(channel, UEs(ueIdx).NumTransmitAntennas,gNBs(i).NumReceiveAntennas,...
                "uplink");
            channel.SampleRate = sampleRate;
            channel.ChannelFiltering = false;
            channels{UEs(ueIdx).ID, gNBs(i).ID} = channel;
        
            % Configure the downlink channel model between gNB and UE
            channel = nrCDLChannel;
            channel.DelayProfile = channelConfig.DelayProfile;
            channel.DelaySpread = channelConfig.DelaySpread;
            channel.Seed = 73 + (ueIdx - 1);
            channel.CarrierFrequency = gNBs(i).CarrierFrequency;
            channel = hArrayGeometry(channel, gNBs(i).NumTransmitAntennas,UEs(ueIdx).NumReceiveAntennas,...
                "downlink");
            channel.SampleRate = sampleRate;
            channel.ChannelFiltering = false;
            channels{gNBs(i).ID, UEs(ueIdx).ID} = channel;
        end
    end
end


