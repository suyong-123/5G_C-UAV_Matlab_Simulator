%% =========================================================================
%  5G UAV Simulation Script
%  - Two UEs and Two gNBs with 2.6 GHz TDD channel
%  - End-to-end UDP socket integration for external QUIC communication
%  - Dynamic UE-port mapping, packet forwarding, and callback-based control
%  - Supports uplink and downlink flow and server ACK feedback
%  Author: [Yong Su]
%  Date: [2025-06-09]
%  =========================================================================

%% Network Initialization
rng("default") % Reset the random number generator
networkSimulator = wirelessNetworkSimulator.init;

% gNB and UE Configuration
gNBPositions = [0 0 25; 350 0 25];  
gNBNames = "gNB-" + (1:size(gNBPositions,1));
gNBs = nrGNB(Name=gNBNames,Position=gNBPositions,CarrierFrequency=2.6e9,ChannelBandwidth=20e6,SubcarrierSpacing=30e3,...
    NumTransmitAntennas=16,NumReceiveAntennas=8,ReceiveGain=11,DuplexMode="TDD");
global numCells
numCells = length(gNBs);

uePositions =[-100 100 100; 400 0 150];
UENames = "UE-" + (1:size(uePositions,1));
UEs = nrUE(Name=UENames,Position=uePositions,NumTransmitAntennas=4,NumReceiveAntennas=2,ReceiveGain=11);
numsUE = length(UEs);

for i = 1:numsUE
    configureULforSRS(UEs(i),gNBs);
end

rlcBearerConfig = nrRLCBearerConfig(SNFieldLength=6,BucketSizeDuration=10); 
% Initial attachment
connectUE(gNBs(1),UEs(1),RLCBearerConfig=rlcBearerConfig);
connectUE(gNBs(2),UEs(2),RLCBearerConfig=rlcBearerConfig);

% Add nodes and channel
addNodes(networkSimulator,gNBs);  
addNodes(networkSimulator,UEs);    

channelConfig = struct("DelayProfile", "CDL-C", "DelaySpread", 100e-9);
channels = createCDLChannels(channelConfig, gNBs, UEs);
customChannelModel = hNRCustomChannelModel(channels,struct(PathlossMethod="nrPathloss"));
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

% Optional mobility
enableMobility = false;
if enableMobility
    ueSpeedRange = [1000 1500]; % UAV speed: 10-15 m/s
    ueWithMobility = UEs; 
    addMobility(ueWithMobility, SpeedRange=ueSpeedRange, BoundaryShape="rectangle", Bounds=[300 150 1000 700]);
end

%% Traffic Configuration
appDataRate = 1e3; % Application data rate in kbps
for ueIdx = 1:numsUE

    ulApps(ueIdx) = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(UEs(ueIdx), ulApps(ueIdx)); 
    dlApps(ueIdx) = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(gNBs(UEs(ueIdx).GNBNodeID), dlApps(ueIdx), DestinationNode=UEs(ueIdx));
end

%% Network Topology Visualization
networkVisualizer = helperNetworkVisualizer(SampleRate=100); 
addNodes(networkVisualizer,gNBs);
addNodes(networkVisualizer,UEs);
showBoundaries(networkVisualizer, gNBPositions, 200); % Show coverage boundaries as circles

%% Mobility Manager (Optional)
% h1 = handoverManager(UEs(1), gNBs, networkSimulator, ulApps(1), dlApps(1), 'train');
% h2 = handoverManager(UEs(2), gNBs, networkSimulator, ulApps(2), dlApps(2), 'train');

%% Register Callbacks for UDP packet forwarding
addlistener(UEs, 'AppDataReceived', @(src, eventData) UEToClient(src, eventData)); 
addlistener(gNBs, 'AppDataReceived', @(src, eventData) GNBToServer(src, eventData)); 

%% Trigger Simulation
triggerFcn(networkSimulator, ulApps, dlApps, UEs);





%% Event Trigger
function triggerFcn(networkSimulator, ulApps, dlApps, UEs)
    % Global variable declarations
    global numCells
    global simulationTime ueSockets udpsocket0;
    numsUE=length(UEs);

    simulationTime = 0;
    maxSimulationTime = 5; % seconds
    ip = '10.92.8.105';  % all sockets use same IP
    
    % Initialize UDP sockets
    % Client → MATLAB (Receive)
    udpsocket0 = udpport("datagram", "LocalHost", ip, "LocalPort", 8000, "Timeout", 100);
    udpsocket0.OutputDatagramSize = 65507;
    
    % UEs(x) → udpsocketx 
    ueSockets = cell(1,numsUE);  % Use cell array to store udpport objects
    for idx=1:numsUE
        ueSockets{idx} = udpport("datagram", "LocalHost",'10.92.8.105',"LocalPort", 8000 + idx, "Timeout", 100);
        ueSockets{idx}.OutputDatagramSize = 65507;
    end

    % Register callbacks
    % udpsocket0 receives data and distributes to UEs
    configureCallback(udpsocket0, "datagram", 1,@(src, ~) distributePacketsToUEs(src, networkSimulator, ulApps, UEs));
    
    % udpsocket1~x receives data and delivers to dlApps(idx), corresponding UE ID is idx+numCells
    for idx = 1:numsUE
        configureCallback(ueSockets{idx}, "datagram", 1, @(src, ~) ServerAckTodlApp(src, networkSimulator,dlApps(idx),idx+numCells));
    end

    disp('[Trigger] Simulation started...');

    % === Main loop to allow callbacks to execute
    while simulationTime < maxSimulationTime
        drawnow;
        pause(0.1);
    end

    % === Cleanup
    configureCallback(udpsocket0, "off");
    for idx=1:numsUE
        configureCallback(ueSockets{idx}, "off");
        clear ueSockets{idx};
    end
    clear udpsocket0;

    disp('[Trigger] Simulation finished');
end


% udpsocket0 receives packets from multiple quic-go clients. All client IPs are the same, but ports differ.
% Identify and bind to n UEs dynamically based on port number,
% and forward data into the corresponding ulApps(idx)
function distributePacketsToUEs(src,networkSimulator,ulApps,UEs)
    global randomPort numCells;
    numsUE=length(UEs);
    if isempty(randomPort)
        randomPort = cell(1,numsUE); 
    end
    if src.NumDatagramsAvailable>0
      data = read(src,src.NumDatagramsAvailable);
      for i=1:length(data)
          sendData=data(1,i).Data; % Row vector
          originalLength=length(sendData); % Original length
          lengthField = typecast(uint16(originalLength), 'uint8'); % Data length field (2 bytes)
          packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)]; % Form row vector

          senderPort=data(1,i).SenderPort;
          % First time binding the port (find the first empty slot)
          if ~any(cellfun(@(p) isequal(p, senderPort), randomPort))
              for idx = 1:numsUE
                  if isempty(randomPort{idx})
                      randomPort{idx} = senderPort;
                      fprintf("[INFO] Mapped senderPort %d -> UE%d\n", senderPort, idx+numCells);
                      break;
                  end
              end
          end

          % Find UE index corresponding to this sender port
          ueIdx = find(cellfun(@(p) isequal(p, senderPort), randomPort), 1);
          if isempty(ueIdx)
              warning("[WARN] Sender port %d not mapped to any UE. Ignored.", senderPort);
              continue;
          end

          % Dispatch to appropriate UE based on port
          ClientToUE(networkSimulator, packetedData, ulApps(ueIdx), UEs(ueIdx));
      end
    end
end

function ClientToUE(networkSimulator,packetedData,ulApp,UE)
    % Enable forwarding flag on corresponding gNB
    networkSimulator.Nodes{UE.GNBNodeID,1}.TrafficManager.isForward=true;
    global simulationTime;
    if isempty(simulationTime)
        simulationTime = 0;
    end
    ulApp.pAppData=double(packetedData'); % ulApp needs column vector
    
    simulationTime = simulationTime + 2*ulApp.pTransmissionTime;
    fprintf("[Time: %.6f] UE%d received packet and wrote to ulApp\n", ...
            networkSimulator.CurrentTime, UE.ID);

    run(networkSimulator, simulationTime);
    fprintf("[ClinetToUE] UE%d forwarded\n",UE.ID);
end

% gNBs forward to server
function GNBToServer(~,event)
    global ueSockets numCells;
    SourceUEID=event.Data.NodeID; % UE that sent this data
    receiveData=event.Data.Packet; % Column vector
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16');
    originalData = receiveData(3:2 + originalLength); % Extract original data
    fprintf("[Time: %.6f] gNB received packet from UE%d (size: %d bytes)\n", ...
            event.Data.CurrentTime, SourceUEID, originalLength);
    targetIP = "10.92.8.105";
    targetPort = 6121;

    socketidx=SourceUEID-numCells;
    socket=ueSockets{socketidx};
    write(socket, originalData', targetIP, targetPort); 
    fprintf("[GNBToServer] udpsocket%d forwarded\n",socketidx);
end

% udpsocket1~x receives server ACK frames and writes into corresponding UE's dlApp
function ServerAckTodlApp(src,networkSimulator,dlApp,targetUEID)
    % udpsocket(i), write to the corresponding dlApp

    networkSimulator.Nodes{targetUEID,1}.TrafficManager.isForward=true;
    global simulationTime numCells; 
    if isempty(simulationTime)
        simulationTime = 0;
    end
    if src.NumDatagramsAvailable>0
        data = read(src,src.NumDatagramsAvailable);
        sendData=data.Data;
        originalLength=length(sendData); % Original length
        % Packet the data
        lengthField = typecast(uint16(originalLength), 'uint8'); % Length field (2 bytes)
        packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)]; % Row vector
        dlApp.pAppData=double(packetedData');

        simulationTime = simulationTime + 2*dlApp.pTransmissionTime;
        run(networkSimulator, simulationTime);
    end
    fprintf("[ServerACKToUE] udpsocket%d receive ACK to UE (size: %d bytes)\n",targetUEID-numCells,originalLength);
end

% UEs forward data
% UEs(i) → udpsocket0 → client(i)
function UEToClient(src, event)
    global udpsocket0 numCells;
    receiveData=event.Data.Packet;
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16'); % Decode first 2 bytes
    originalData = receiveData(3:2 + originalLength); % Extract original data

    % Forward to client
    global randomPort;
    ip = "10.92.8.105";
    port=randomPort{src.ID-numCells};
    write(udpsocket0, originalData', ip, port); 
    fprintf("[UEToClient] UE%d forwarded\n",src.ID);
end

%% Cleanup function

function cleanupUDPPorts()
    % Clean up global UDP socket objects
    global udpsocket0 ueSockets;
    % Turn off udpsocket0 callback and clear
    if ~isempty(udpsocket0)
        configureCallback(udpsocket0, "off");
        clear udpsocket0;
    end
    % Turn off and clear each ueSocket's callback  
    for i = 1:length(ueSockets)
        if ~isempty(ueSockets{i})
            configureCallback(ueSockets{i}, "off");
            clear ueSockets(i);
        end
    end
    % Clear global variables
    clear global udpsocket0 ueSockets;
    disp('[cleanupUDPPorts] All UDP ports cleaned up.');
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


