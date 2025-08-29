% tcp+matlab
%% nrNode
rng("default") % Reset the random number generator
numFrameSimulation = 10; % Simulation time in terms of number of 10 ms frames
networkSimulator = wirelessNetworkSimulator.init;
% nrNode

gNBPositions = [0 500 30; 500 500 30; 250 1000 30];   

gNBs = nrGNB(Position=gNBPositions,CarrierFrequency=2.6e9,ChannelBandwidth=20e6,SubcarrierSpacing=30e3,...
    NumTransmitAntennas=16,NumReceiveAntennas=8,ReceiveGain=11,DuplexMode="TDD");


uePositions =[0 400 150];
UEs = nrUE(Position=uePositions,NumTransmitAntennas=4,NumReceiveAntennas=2,ReceiveGain=11);


rlcBearerConfig = nrRLCBearerConfig(SNFieldLength=6,BucketSizeDuration=10);
connectUE(gNBs(1),UEs,RLCBearerConfig=rlcBearerConfig);

enableMobility = false;
if enableMobility
    ueSpeedRange = [1000 1500]; % In meters per second
    ueWithMobility = UEs; % Get UE-2 in the cell of interest
    % Add random waypoint mobility to the selected UE
    addMobility(ueWithMobility,SpeedRange=ueSpeedRange,BoundaryShape="R",Bounds=100)
end

addNodes(networkSimulator,gNBs);  
addNodes(networkSimulator,UEs)

channelConfig = struct("DelayProfile", "CDL-C", "DelaySpread", 100e-9);
channels = createCDLChannels(channelConfig, gNBs, UEs);
customChannelModel = hNRCustomChannelModel(channels,struct(PathlossMethod="nrPathloss"));
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

% networkVisualizer = helperNetworkVisualizer(SampleRate=100); % Sample rate indicates the visualization refresh rate in Hertz
% addNodes(networkVisualizer,gNBs);
% addNodes(networkVisualizer,UEs);
% 
% %Show the cell boundary of each gNB
% showBoundaries(networkVisualizer,gNBPositions,1000)

appDataRate = 1e3; % Application data rate in kilo bits per second (kbps) 
% UL
ulApps=networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
addTrafficSource(UEs,ulApps);
% DL
dlApps = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
addTrafficSource(gNBs(1),dlApps,DestinationNode=UEs);

addlistener(gNBs, 'AppDataReceived', @(src, eventData) forwardULPacketFromPrimaryPath(src, eventData)); 

%% run
triggerFcn(networkSimulator,ulApps);


function triggerFcn(networkSimulator, ulApps)
    global simulationTime tcpsocket1 tcpsocket2;
    simulationTime = 0;
    ip = '10.92.8.105';
    
    % init TCP socket
    try
        tcpsocket1 = tcpserver(ip,8001, "Timeout", 100);
        configureCallback(tcpsocket1, 'byte',1500, @(src,~) ToUECallback(src, networkSimulator, ulApps));
        disp('TCP Server started on port 8001');
    catch ME
        error('Failed to start server: %s', ME.message);
    end
    
    % connect to target TCP server
    try
        tcpsocket2 = tcpclient(ip, 1234, "Timeout", 10);
        disp('Connected to target server');
    catch
        warning('Failed to connect to target server');
    end

    while isvalid(tcpsocket1) && tcpsocket1.Connected
        pause(1);
    end
    
    disp('Press any key to stop ...');
    pause;
    
    % Clear callback functions and close servers
    configureCallback(tcpsocket1, "off");
    configureCallback(tcpsocket2, "off");
 
    
    clear tcpsocket1 
    clear tcpsocket2
    disp('finish');
end

%% callback
function ToUECallback(src, networkSimulator, ulApps)
    try
        if src.NumBytesAvailable > 0
       
            rawData = read(src, src.NumBytesAvailable, 'uint8');

            originalLength = uint16(length(rawData));
            lengthField = typecast(uint16(originalLength), 'uint8');
            packetedData = [lengthField, rawData];

            distributePacketsToUEs(packetedData, networkSimulator, ulApps, 1);
            disp('[tcpsocket2] Forwarded to TCP Server')
            
            
        end
    catch ME
        disp(['Callback error: ', ME.message]);
    end
end

function distributePacketsToUEs(packetedData,networkSimulator,ulApp,x)

    networkSimulator.Nodes{x,1}.TrafficManager.isForward=true;
    global simulationTime; % Use persistent variable to store simulation time
    if isempty(simulationTime)
        simulationTime = 0; % Initialize simulationTime as the initial time
    end
    ulApp.pAppData=double(packetedData'); 
    
    simulationTime = simulationTime + ulApp.pTransmissionTime;
    run(networkSimulator, simulationTime);
   


end

function forwardULPacketFromPrimaryPath(~,event)
    global tcpsocket2;
    receiveData=event.Data.Packet; 
    originalLength = typecast(uint8(receiveData(1:2)), 'uint16');
    originalData = receiveData(3:2 + originalLength);
    write(tcpsocket2, originalData','uint8'); 
    
    disp('[tcpsocket1] Received packet from TCP Client');
   % fprintf('TotalLatency: %.4f\n',src.TrafficManager.StatTotalLatency)


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


