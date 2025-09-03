%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Description:
%   This MATLAB script performs testing of a trained DDPG\DQN-based
%   intelligent handover algorithm in a 5G NR network simulation.
%   The scenario consists of one UAV-mounted UE and five gNBs with
%   mobility enabled. The simulation runs multiple episodes, during which
%   uplink and downlink traffic are generated and SINR, throughput, and
%   handover events are recorded.
%
% Features:
%   one UE and five gNBs
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 pe = pyenv('Version', 'D:\anaconda3\envs\MATLABProj\python.exe');
if count(py.sys.path, pwd) == 0
    insert(py.sys.path, int32(0), pwd);
end
py.importlib.import_module('DDQN');
num_episodes=11;
DQN_nums=zeros(num_episodes,1);
DQN_SINR=cell(num_episodes,1);
DQN_throughput=cell(2,num_episodes);

for ep=1:num_episodes
   
    rng("shuffle") % Reset the random number generator
    numFrameSimulation = 10; % Simulation time in terms of number of 10 ms frames
    networkSimulator = wirelessNetworkSimulator.init;
   
    gNBPositions = [0 0 25; 350 0 25;700 0 25;180 300 25; 550 300 25 ];  

    gNBNames = "gNB-" + (1:size(gNBPositions,1));
    gNBs = nrGNB(Name=gNBNames,Position=gNBPositions,CarrierFrequency=2.6e9,ChannelBandwidth=20e6,SubcarrierSpacing=30e3,...
        NumTransmitAntennas=16,NumReceiveAntennas=8,ReceiveGain=11,DuplexMode="TDD");
    uePositions =[400 200 150]; %
    UENames = "UE-" + (1:size(uePositions,1));
    UE = nrUE(Name=UENames,Position=uePositions,NumTransmitAntennas=4,NumReceiveAntennas=2,ReceiveGain=11);
    
    configureULforSRS(UE,gNBs);
    rlcBearerConfig = nrRLCBearerConfig(SNFieldLength=6,BucketSizeDuration=10); 
    connectUE(gNBs(2),UE,RLCBearerConfig=rlcBearerConfig);%UE初始和gNBs(3)相连

    addNodes(networkSimulator,gNBs);  
    addNodes(networkSimulator,UE)    
    %channel
    channelConfig = struct("DelayProfile", "CDL-C", "DelaySpread", 100e-9);
    channels = createCDLChannels(channelConfig, gNBs, UE);
    customChannelModel = hNRCustomChannelModel(channels,struct(PathlossMethod="nrPathloss"));
    addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

    enableMobility = true;
    if enableMobility
        ueSpeedRange = [1000 1500]; % In meters per second 无人机飞行速度为10-15m/s
        ueWithMobility = UE; 
        % Add random waypoint mobility to the selected UE
        addMobility(ueWithMobility,SpeedRange=ueSpeedRange,BoundaryShape="rectangle",Bounds=[400 200 800 600]) %400 200为中心点，800 600为长宽
    end
    
    %% Traffic Configuration
    
    appDataRate = 1e3; % Application data rate in kilo bits per second (kbps) 
    % UL
    ulApps=networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(UE,ulApps); 
    
    % DL
    
    dlApps = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(gNBs(2),dlApps,DestinationNode=UE);
    
    
    %% 网络拓扑图 
    networkVisualizer = helperNetworkVisualizer(SampleRate=100); % Sample rate indicates the visualization refresh rate in Hertz
    addNodes(networkVisualizer,gNBs);
    addNodes(networkVisualizer,UE);
    
    showBoundaries(networkVisualizer,gNBPositions,200);%圆形
    
    
    %% 移动性管理
    handover=handoverManager(UE,gNBs, networkSimulator,ulApps,dlApps,'eval');
    
    
    %% 监听事件
    addlistener(UE, 'AppDataReceived', @(src, eventData) UEToClient(src, eventData)); 
    addlistener(gNBs, 'AppDataReceived', @(src, eventData) GNBToServer(src, eventData)); 
    
    
    %% run
    triggerFcn(networkSimulator,ulApps,dlApps,UE);
    disp(handover.handover_num); 
    cleanupUDPPorts();
    %handover.handover_num、handover.connectedulSINR(UE.SINR)、handover.allthroughput
    DQN_nums(ep)=handover.handover_num;
    DQN_SINR{ep}=handover.connectedulSINR;
    DQN_throughput{2,ep}=handover.allthroughput;
    DQN_throughput{1,ep}=handover.allthroughput_time;
end
save('evaDQN_5_19.mat','DQN_throughput','DQN_SINR',"DQN_nums");

%% 事件触发
function triggerFcn(networkSimulator, ulApps, dlApps, UE)
    global simulationTime udpsocket1 udpsocket2;
    simulationTime = 0;
    maxSimulationTime = 2.3; 
    ip = '10.92.8.105';
    udpsocket1 = udpport("datagram", "LocalHost", ip, "LocalPort", 8000, "Timeout", 100); 
    udpsocket1.OutputDatagramSize = 65507;
    udpsocket2 = udpport("datagram", "LocalHost", ip, "LocalPort", 8004, "Timeout", 100);
    udpsocket2.OutputDatagramSize = 65507;

    configureCallback(udpsocket1, "datagram", 1, @(src, ~) ClientToUE(src, networkSimulator, ulApps, UE));
    configureCallback(udpsocket2, "datagram", 1, @(src, ~) ServerAckToGNB(src, networkSimulator, dlApps, UE.ID));
    disp('start...');
   
    while simulationTime < maxSimulationTime
        drawnow; 
        pause(0.1); 
    end
    
    configureCallback(udpsocket1, "off");
    configureCallback(udpsocket2, "off");
    clear udpsocket1
    clear udpsocket2
    disp('[Trigger] Simulation finished');
end



function ClientToUE(src,networkSimulator,ulApps,UE)
    
    networkSimulator.Nodes{UE.GNBNodeID,1}.TrafficManager.isForward=true;

    global randomPort1 simulationTime;
    if isempty(simulationTime)
        simulationTime = 0; % Initialize simulationTime as the initial time
    end

    if src.NumDatagramsAvailable>0
        data = read(src,src.NumDatagramsAvailable);
        for i=1:length(data)
            sendData=data(1,i).Data;
            originalLength=length(sendData);
            randomPort1=data(1,i).SenderPort;
            lengthField = typecast(uint16(originalLength), 'uint8'); 
            packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)]; 
        

            ulApps.pAppData=double(packetedData'); 
            
            simulationTime = simulationTime + ulApps.pTransmissionTime;
            run(networkSimulator, simulationTime);

        end

    end


end


function GNBToServer(~,event)
    global udpsocket2;
    receiveData=event.Data.Packet; 
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16');
    originalData = receiveData(3:2 + originalLength);
    write(udpsocket2, originalData', "10.92.8.105", 6121); 
    disp('gNB1 forwarded')
    %fprintf('TotalLatency: %.4f\n',src.TrafficManager.StatTotalLatency)


end

function ServerAckToGNB(src,networkSimulator,dlApp,x)
    

    networkSimulator.Nodes{x,1}.TrafficManager.isForward=true;
    global simulationTime; 
    if isempty(simulationTime)
        simulationTime = 0; % Initialize simulationTime as the initial time
    end
    if src.NumDatagramsAvailable>0
        data = read(src,src.NumDatagramsAvailable);
        
        sendData=data.Data;
        originalLength=length(sendData);
        lengthField = typecast(uint16(originalLength), 'uint8'); 
        packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)]; 
        dlApp.pAppData=double(packetedData');
        
        simulationTime = simulationTime + dlApp.pTransmissionTime; % Accumulate simulation time
        run(networkSimulator, simulationTime);
          
    end
    disp('server ack');

end


function UEToClient(~, event)
    global udpsocket1;
    receiveData=event.Data.Packet;
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16'); 
    originalData = receiveData(3:2 + originalLength);
   
    % Forward
    global randomPort1;
    ip1 = "10.92.8.105";
    port1 = randomPort1;
    write(udpsocket1, originalData', ip1, port1); 
    disp('UE1 forwarded')
    


end








function cleanupUDPPorts()
    global udpsocket1 udpsocket2;
    if exist('udpsocket1', 'var') && isvalid(udpsocket1)
        configureCallback(udpsocket1, "off");
        delete(udpsocket1);
        clear udpsocket1;
        disp('已清理udpsocket1');
    end

    if exist('udpsocket2', 'var') && isvalid(udpsocket2)
        configureCallback(udpsocket2, "off");
        delete(udpsocket2);
        clear udpsocket2;
        disp('已清理udpsocket2');
    end
    

    java.lang.System.gc(); 
    pause(0.5); 
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


