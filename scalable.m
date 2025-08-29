%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Description:
%   This MATLAB script simulates a 5G NR network scenario with multiple UE and
%   three gNBs where no handover is performed throughout the simulation.
%   The focus is on testing the system's extensibility and stability under
%   UDP-based data interactions only, without mobility-triggered handovers.
%
% Features:
%   - Static UE-to-gNB connection (no handover events)
%   - UDP communication for uplink simulation
%   - Simplified scenario for verifying UDP data flow and system modularity
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rng("default")
%numFrameSimulation = 10; % Simulation time in terms of number of 10 ms frames
networkSimulator = wirelessNetworkSimulator.init;
gNBPositions = [0 500 30; 500 500 30; 250 1000 30];      

gNBNames = "gNB-" + (1:size(gNBPositions,1));
gNBs = nrGNB(Name=gNBNames,Position=gNBPositions,CarrierFrequency=2.6e9,ChannelBandwidth=20e6,SubcarrierSpacing=30e3,...
    NumTransmitAntennas=16,NumReceiveAntennas=8,ReceiveGain=11,DuplexMode="TDD");

numCells = numel(gNBs);
cellRadius = 200; % Radius of each cell (in meters)
numUEsPerCell = 6;
uePositions = generateUEPositions(cellRadius,gNBPositions,numUEsPerCell);


rlcBearerConfig = nrRLCBearerConfig(SNFieldLength=6,BucketSizeDuration=10); 
UECells = cell(numCells,1);
for cellIdx = 1:numCells
    ueNames = "UE-" + (1:size(uePositions{cellIdx},1));
    UECells{cellIdx} = nrUE(Name=ueNames,Position=uePositions{cellIdx},ReceiveGain=11);
    connectUE(gNBs(cellIdx),UECells{cellIdx},RLCBearerConfig=rlcBearerConfig)
end
UEs = cat(2, UECells{:});
numsUE=length(UEs);



for i=1:numsUE
    configureULforSRS(UEs(i),gNBs);
end

addNodes(networkSimulator,gNBs);  
addNodes(networkSimulator,UEs)    
%channel

channelConfig = struct("DelayProfile", "CDL-C", "DelaySpread", 100e-9);
channels = createCDLChannels(channelConfig, gNBs, UEs);
customChannelModel = hNRCustomChannelModel(channels,struct(PathlossMethod="nrPathloss"));
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

appDataRate = 5e3; % Application data rate in kilo bits per second (kbps) 
for ueIdx=1:numsUE 
    ulApps(ueIdx)=networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(UEs(ueIdx),ulApps(ueIdx)); 
    
    % dlApps(ueIdx) = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    % addTrafficSource(gNBs(UEs(ueIdx).GNBNodeID),dlApps(ueIdx),DestinationNode=UEs(ueIdx));
end

networkVisualizer = helperNetworkVisualizer(SampleRate=100); % Sample rate indicates the visualization refresh rate in Hertz
% addNodes(networkVisualizer,gNBs);
% addNodes(networkVisualizer,UEs);
% 
% showBoundaries(networkVisualizer,gNBPositions,200);%圆形


%%
%addlistener(UEs, 'AppDataReceived', @(src, eventData) UEToClient(src, eventData)); 
addlistener(gNBs, 'AppDataReceived', @(src, eventData) GNBToServer(src, eventData)); 


%% run
%run(networkSimulator,2);
triggerFcn(networkSimulator,ulApps,[],UEs);



%% 事件触发
function triggerFcn(networkSimulator, ulApps, dlApps, UEs)
    % Global variable declarations
    global simulationTime ueSockets udpsocket0;
    numsUE=length(UEs);

    simulationTime = 0;
    maxSimulationTime = 100; % seconds
    ip = '10.92.8.105';  % all sockets use same IP
    

    % Client → MATLAB (Receive)
    udpsocket0 = udpport("datagram", "LocalHost", ip, "LocalPort", 8000, "Timeout", 100);
    udpsocket0.OutputDatagramSize = 65507;
    % UEs(x) ->udpsocketx 
    ueSockets = cell(1,numsUE);  
    for idx=1:numsUE
        ueSockets{idx} = udpport("datagram", "LocalHost",'10.92.8.105',"LocalPort", 8000 + idx, "Timeout", 100);
        ueSockets{idx}.OutputDatagramSize = 65507;
    end

    configureCallback(udpsocket0, "datagram", 1,@(src, ~) distributePacketsToUEs(src, networkSimulator, ulApps, UEs));
    
    for idx = 1:numsUE
        configureCallback(ueSockets{idx}, "datagram", 1, @(src, ~) ServerAckTodlApp(src, networkSimulator,dlApps(idx),idx+3));
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


function distributePacketsToUEs(src,networkSimulator,ulApps,UEs)
    global randomPort;
    numsUE=length(UEs);
    if isempty(randomPort)
        randomPort = cell(1,numsUE); 
    end
    if src.NumDatagramsAvailable>0
      data = read(src,src.NumDatagramsAvailable);
      
     
      for i=1:length(data)
          sendData=data(1,i).Data; 
          originalLength=length(sendData);
          lengthField = typecast(uint16(originalLength), 'uint8'); 
          packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)];

          senderPort=data(1,i).SenderPort;
          
            if ~any(cellfun(@(p) isequal(p, senderPort), randomPort))
                for idx = 1:numsUE
                    if isempty(randomPort{idx})
                        randomPort{idx} = senderPort;
                        fprintf("[INFO] Mapped senderPort %d -> UE%d\n", senderPort, idx+3);
                        break;
                    end
                end
            end

            ueIdx = find(cellfun(@(p) isequal(p, senderPort), randomPort), 1);
            if isempty(ueIdx)
                warning("[WARN] Sender port %d not mapped to any UE. Ignored.", senderPort);
                continue;
            end
         
         
          dt = datetime('now');
          dt.Format = 'HH:mm:ss'; 
          fprintf('UE%d接收时间：%s\n',ueIdx+3, dt);  
          ClientToUE(networkSimulator, packetedData, ulApps(ueIdx), UEs(ueIdx));
          
                 
      end
      
    end

end

function ClientToUE(networkSimulator,packetedData,ulApp,UE)
    networkSimulator.Nodes{UE.GNBNodeID,1}.TrafficManager.isForward=true;
    global simulationTime; % Use persistent variable to store simulation time
    if isempty(simulationTime)
        simulationTime = 0; % Initialize simulationTime as the initial time
    end
    ulApp.pAppData=double(packetedData'); 
    
    simulationTime = simulationTime + ulApp.pTransmissionTime;
    %fprintf("[Time: %.6f] UE%d received packet and wrote to ulApp\n", ...
            %networkSimulator.CurrentTime, UE.ID);

    run(networkSimulator, simulationTime);
   % fprintf("[ClinetToUE] UE%d forwarded\n",UE.ID);

end

% gNBs 转发给 server
% 
function GNBToServer(src,event)
    global ueSockets;
    SourceUEID=event.Data.NodeID; 
    receiveData=event.Data.Packet; 
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16');
    originalData = receiveData(3:2 + originalLength);
   % fprintf("[Time: %.6f] gNB received packet from UE%d (size: %d bytes)\n", ...
            %event.Data.CurrentTime, SourceUEID, originalLength);
    targetIP = "10.92.8.105";
    targetPort = 8080;
  
    socketidx=SourceUEID-3;
    socket=ueSockets{socketidx};
    dt = datetime('now');
    dt.Format = 'HH:mm:ss';  
    fprintf('UE%d 发送时间：%s\n',SourceUEID, dt); 
    simTime=src.TrafficManager.packetLatency;
    fprintf('UE%d 仿真运行时间: %.4f\n',SourceUEID,simTime);
   
    write(socket, originalData', targetIP, targetPort); 
    %fprintf("[GNBToServer] udpsocket%d forwarded\n",socketidx);
end


% udp no ACK
function ServerAckTodlApp(src,networkSimulator,dlApp,targetUEID)
  
  
    networkSimulator.Nodes{targetUEID,1}.TrafficManager.isForward=true;
    global simulationTime; 
    if isempty(simulationTime)
        simulationTime = 0; % Initialize simulationTime as the initial time
    end
    if src.NumDatagramsAvailable>0
        data = read(src,src.NumDatagramsAvailable);
        
        sendData=data.Data;
        originalLength=length(sendData);% 原始长度
        %打包
        lengthField = typecast(uint16(originalLength), 'uint8'); % 数据长度字段 (2 字节)
        packetedData=[lengthField,sendData,zeros(1,1500-originalLength-2)]; %行向量
        dlApp.pAppData=double(packetedData');
        
        simulationTime = simulationTime + 3*dlApp.pTransmissionTime; % Accumulate simulation time
        run(networkSimulator, simulationTime);
          
    end
   % fprintf("[ServerACKToUE] udpsocket%d receive ACK to UE\n",targetUEID-3);

end


%%
function uePositions = generateUEPositions(cellRadius,gNBPositions,numUEsPerCell)
%generateUEPositions Return the position of UE nodes in each cell

numCells = size(gNBPositions,1);
uePositions = cell(numCells,1);
ueHeight = 3; % In meters
for cellIdx=1:numCells
    gnbXCo = gNBPositions(cellIdx,1); % gNB X-coordinate
    gnbYCo = gNBPositions(cellIdx,2); % gNB Y-coordinate
    theta = rand(numUEsPerCell,1)*(2*pi);
    % Use these expressions to calculate the position of UE nodes within the cell. By default,
    % the placement of the UE nodes is random within the cell
    r = sqrt(rand(numUEsPerCell,1))*cellRadius;
    x = round(gnbXCo + r.*cos(theta));
    y = round(gnbYCo + r.*sin(theta));
    z = ones(numUEsPerCell,1) * ueHeight;
    uePositions{cellIdx} = [x y z];
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


