%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: mpquic_matlab_simulation.m
%
% Description:
%   This MATLAB script sets up a simulation environment to test MP-QUIC 
%   (Multipath QUIC) over a 5G NR network using two sets of gNBs and two UEs.
%   Each UE connects to a separate gNB system and communicates via UDP.
%
% Features:
%   - Two independent 5G base station clusters with different frequencies
%   - Two UEs initialized at fixed positions with mobility enabled
%   - CDL-C channel model with path loss
%   - Suitable for testing MP-QUIC data transmission across multiple paths
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% MP-QUIC + MATLAB Integration

%% Network Initialization
rng("default") % Reset random seed for reproducibility
numFrameSimulation = 10; % Simulate 10 frames (10 ms each)
networkSimulator = wirelessNetworkSimulator.init;

% Define gNB positions
% Two separate gNB clusters (e.g., representing two operators)
gNBPositions_1 = [0 0 25; 700 100 25; 900 450 25];     % Cluster 1 (e.g., Mobile)
gNBPositions_2 = [350 0 25; 200 350 25; 550 400 25];   % Cluster 2 (e.g., Telecom)

% Combine positions for full gNB list
gNBPositions = [gNBPositions_1; gNBPositions_2];

% Cluster 1: 2.6 GHz band
gNBNames_1 = "gNB-" + (1:size(gNBPositions_1,1));
gNBs_1 = nrGNB(Name=gNBNames_1, Position=gNBPositions_1, CarrierFrequency=2.6e9, ...
    ChannelBandwidth=20e6, SubcarrierSpacing=30e3, ...
    NumTransmitAntennas=16, NumReceiveAntennas=8, ReceiveGain=11, DuplexMode="TDD");

% Cluster 2: 3.5 GHz band
gNBNames_2 = "gNB-" + (size(gNBPositions_1,1)+1 : size(gNBPositions_1,1)+size(gNBPositions_2,1));
gNBs_2 = nrGNB(Name=gNBNames_2, Position=gNBPositions_2, CarrierFrequency=3.5e9, ...
    ChannelBandwidth=20e6, SubcarrierSpacing=30e3, ...
    NumTransmitAntennas=16, NumReceiveAntennas=8, ReceiveGain=11, DuplexMode="TDD");

% Combine both clusters
gNBs = [gNBs_1, gNBs_2];

% Define UE positions (e.g., UAVs)
uePositions = [180 0 150; 185 0 150];
UENames = "UE-" + (1:size(uePositions,1));
UEs = nrUE(Name=UENames, Position=uePositions, NumTransmitAntennas=4, NumReceiveAntennas=2, ReceiveGain=11);

% Connect UEs to corresponding gNB clusters
rlcBearerConfig = nrRLCBearerConfig(SNFieldLength=6, BucketSizeDuration=10);
connectUE(gNBs_1(1), UEs(1), RLCBearerConfig=rlcBearerConfig); % UE1 connects to gNBs_1
connectUE(gNBs_2(1), UEs(2), RLCBearerConfig=rlcBearerConfig); % UE2 connects to gNBs_2

% Store Cell IDs and UE IDs for later use
cellID_1 = UEs(1).GNBNodeID;
cellID_2 = UEs(2).GNBNodeID;
UEID_1 = UEs(1).ID;
UEID_2 = UEs(2).ID;

% Add all nodes to the simulator
addNodes(networkSimulator, gNBs);
addNodes(networkSimulator, UEs);

%% Channel Model
channelConfig = struct("DelayProfile", "CDL-C", "DelaySpread", 100e-9);
channels = createCDLChannels(channelConfig, gNBs, UEs);
customChannelModel = hNRCustomChannelModel(channels, struct(PathlossMethod="nrPathloss"));
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

%% Mobility Configuration
enableMobility = true;
if enableMobility
    ueSpeedRange = [1 1000]; % Speed range in meters per second
    ueWithMobility = UEs;
    % Add random waypoint mobility model
    addMobility(ueWithMobility, SpeedRange=ueSpeedRange, BoundaryShape="rectangle", Bounds=[300 150 1000 700]);
end

%%

networkVisualizer = helperNetworkVisualizer(SampleRate=100); % Sample rate indicates the visualization refresh rate in Hertz
addNodes(networkVisualizer,gNBs);
addNodes(networkVisualizer,UEs);
cellsOfInterest = [1,2,3];
showBoundaries(networkVisualizer,gNBPositions,200,cellsOfInterest);%圆形


%% Traffic Configuration

appDataRate = 1e3; % Application data rate in kilo bits per second (kbps) 
% UL
for ueIdx=1:length(UEs) 
    ulApps(ueIdx)=networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(UEs(ueIdx),ulApps(ueIdx)); 
     
    dlApps(ueIdx) = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(gNBs(UEs(ueIdx).GNBNodeID),dlApps(ueIdx),DestinationNode=UEs(ueIdx));
end


%%
% addlistener(gNBs(cellID_1), 'PacketReceptionEnded', @(src, eventData) displayInfo_UL(src, eventData)); 
% addlistener(UEs(1), 'PacketReceptionEnded', @(src, eventData) displayInfo_DL(src, eventData)); 
addlistener(UEs(1), 'AppDataReceived', @(src, eventData) forwardDLPacketToPrimaryClient(src, eventData)); 
addlistener(UEs(2), 'AppDataReceived', @(src, eventData) forwardDLPacketToSecondaryClient(src, eventData)); 
addlistener(gNBs(cellID_1), 'AppDataReceived', @(src, eventData) forwardULPacketFromPrimaryPath(src, eventData)); 
addlistener(gNBs(cellID_2), 'AppDataReceived', @(src, eventData) forwardULPacketFromSecondaryPath(src, eventData)); 




%% run
triggerFcn(networkSimulator,ulApps,dlApps,cellID_1,cellID_2,UEID_1,UEID_2);


%% 
function triggerFcn(networkSimulator,ulApps,dlApps,cellID_1,cellID_2,UEID_1,UEID_2)
    global simulationTime udpsocket1 udpsocket2 udpsocket3;
    simulationTime=0;
   
    ip='10.92.8.105';
    udpsocket1 = udpport("datagram", "LocalHost", ip, "LocalPort", 8000, "Timeout", 100); 
    udpsocket1.OutputDatagramSize = 65507;
    udpsocket2 = udpport("datagram", "LocalHost", ip, "LocalPort", 8004, "Timeout", 100);
    udpsocket2.OutputDatagramSize=65507;
    udpsocket3 = udpport("datagram", "LocalHost", ip, "LocalPort", 8005, "Timeout", 100);
    udpsocket3.OutputDatagramSize=65507;
 disp('start...');
    % Pass networkSimulator and simulationTime to the callback function using an anonymous function
    configureCallback(udpsocket1, "datagram", 1, @(src,~) MultipathPartitionCallback(src,networkSimulator,ulApps,cellID_1,cellID_2));
    configureCallback(udpsocket2, "datagram", 1, @(src,~) processServerAckCallback(src,networkSimulator,dlApps(1),UEID_1));
    configureCallback(udpsocket3, "datagram", 1, @(src,~) processServerAckCallback(src,networkSimulator,dlApps(2),UEID_2));
       

    disp('Press any key to stop ...');
    pause;
    
    % Clear callback functions and close servers
    configureCallback(udpsocket1, "off");
    configureCallback(udpsocket2, "off");
    configureCallback(udpsocket3, "off");
    
    clear udpsocket1 
    clear udpsocket3 
    clear udpsocket2
    disp('finish');

end

%% 
function MultipathPartitionCallback(src,networkSimulator,ulApps,cellID_1,cellID_2)
    global randomPort1 randomPort2;
    ip1="10.92.8.105"; %main path
    ip2="10.92.8.83";
    if src.NumDatagramsAvailable>0
      data = read(src,src.NumDatagramsAvailable);
      for i=1:length(data)
          sendData=data(1,i).Data; 
          originalLength=length(sendData);
          sourceIP=data(1,i).SenderAddress;
          
          if  sourceIP==ip1
              randomPort1=data(1,i).SenderPort;
          elseif sourceIP==ip2
              randomPort2=data(1,i).SenderPort;
          end
          lengthField = typecast(uint16(originalLength), 'uint8'); 
          packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)]; 
          
          if sourceIP==ip1
              disp('[udpsocket1] Received packet from client')
              distributePacketsToUEs(packetedData,networkSimulator,ulApps(1),cellID_1)
          elseif sourceIP==ip2
              disp('[udpsocket1] Received packet from 10.92.8.83 (Path 2)')
              distributePacketsToUEs(packetedData,networkSimulator,ulApps(2),cellID_2)
          end
      end
      
    end

end


function distributePacketsToUEs(packetedData,networkSimulator,ulApp,cellID)
  
    networkSimulator.Nodes{cellID,1}.TrafficManager.isForward=true;
    global simulationTime; % Use persistent variable to store simulation time
    if isempty(simulationTime)
        simulationTime = 0; % Initialize simulationTime as the initial time
    end
    ulApp.pAppData=double(packetedData'); 
    
    simulationTime = simulationTime + ulApp.pTransmissionTime;
    run(networkSimulator, simulationTime);


end

% gNB1 forward
function forwardULPacketFromPrimaryPath(~,event)
    global udpsocket2;
    receiveData=event.Data.Packet; 
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16');
    originalData = receiveData(3:2 + originalLength);
    write(udpsocket2, originalData', "10.92.8.105", 6121); 
    disp('[udpsocket2] Forwarded packet from UE-1 to QUIC Server ')
    %fprintf('TotalLatency: %.4f\n',src.TrafficManager.StatTotalLatency)


end
% gNB2 forward
function forwardULPacketFromSecondaryPath(~,event)
    global udpsocket3;
    receiveData=event.Data.Packet;
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16');
    originalData = receiveData(3:2 + originalLength);
    write(udpsocket3, originalData', "10.92.8.105", 6121); 
    disp('[udpsocket3] Forwarded packet from UE-2 to QUIC Server');
end


% udpsocket2\udpsocket3 receive server ack
function processServerAckCallback(src,networkSimulator,dlApp,UEID)
   
   
    networkSimulator.Nodes{UEID,1}.TrafficManager.isForward=true;
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

end

% UE1 forward
function forwardDLPacketToPrimaryClient(~, event)
    global udpsocket1;
    receiveData=event.Data.Packet;
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16'); 
    originalData = receiveData(3:2 + originalLength);
    % Forward
    global randomPort1;
    ip1 = "10.92.8.105";
    port1 = randomPort1;
    write(udpsocket1, originalData', ip1, port1); 
    disp('[udpsocket1] Forwarded ACK to Client-1')


end
% UE2 forward
function forwardDLPacketToSecondaryClient(~, event)
    global udpsocket1;
    receiveData=event.Data.Packet;
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16'); 
    originalData = receiveData(3:2 + originalLength);
   
    % Forward
    global randomPort2;
    ip = "10.92.8.83";
    port = randomPort2;
    write(udpsocket1, originalData', ip, port); 
    disp('[udpsocket1] Forwarded ACK to Client-2')


end



function displayInfo_UL(~,event)
    if event.Data.SignalType=="PUSCH"
        disp(['PUSCH-SINR: ',num2str(event.Data.SINR, '%.2f')])
    end
    if event.Data.SignalType=="SRS"
        disp(['SRS-SINR: ',num2str(event.Data.ChannelMeasurements.SINR, '%.2f')])
    end

end

function displayInfo_DL(~,event)
    if event.Data.SignalType=="PDSCH"
        disp(['PDSCH-SINR: ',num2str(event.Data.SINR, '%.2f')])
    end

    if event.Data.SignalType=="CSIRS"
        disp(['CSIR-SINR: ',num2str(event.Data.ChannelMeasurements.SINR, '%.2f')])
        disp(['CSIR-CQI: ',num2str(event.Data.ChannelMeasurements.CQI)])
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


