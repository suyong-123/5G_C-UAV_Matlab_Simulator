% no handover no socket

%% network
rng("default") % Reset the random number generator
%numFrameSimulation = 10; % Simulation time in terms of number of 10 ms frames
networkSimulator = wirelessNetworkSimulator.init;

% nrNode
gNBPositions = [0 0 25; 350 0 25;700 0 25;180 300 25; 550 300 25 ];      
gNBNames = "gNB-" + (1:size(gNBPositions,1));
gNBs = nrGNB(Name=gNBNames,Position=gNBPositions,CarrierFrequency=2.6e9,ChannelBandwidth=20e6,SubcarrierSpacing=30e3,...
    NumTransmitAntennas=16,NumReceiveAntennas=8,ReceiveGain=11,DuplexMode="TDD");


uePositions =[-100 100 100;400 0 150]; %650 300 
UENames = "UE-" + (1:size(uePositions,1));
UEs= nrUE(Name=UENames,Position=uePositions,NumTransmitAntennas=4,NumReceiveAntennas=2,ReceiveGain=11);
numsUE=length(UEs);

for i=1:numsUE
    configureULforSRS(UEs(i),gNBs);
end

rlcBearerConfig = nrRLCBearerConfig(SNFieldLength=6,BucketSizeDuration=10); 
% first connect
connectUE(gNBs(1),UEs(1),RLCBearerConfig=rlcBearerConfig);
connectUE(gNBs(2),UEs(2),RLCBearerConfig=rlcBearerConfig);


%
addNodes(networkSimulator,gNBs);  
addNodes(networkSimulator,UEs)    
%channel

channelConfig = struct("DelayProfile", "CDL-C", "DelaySpread", 100e-9);
channels = createCDLChannels(channelConfig, gNBs, UEs);
customChannelModel = hNRCustomChannelModel(channels,struct(PathlossMethod="nrPathloss"));
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);


enableMobility = false;
if enableMobility
    ueSpeedRange = [1000 1500]; % In meters per second 10-15m/s
    ueWithMobility = UEs; 
    % Add random waypoint mobility to the selected UE
    addMobility(ueWithMobility,SpeedRange=ueSpeedRange,BoundaryShape="rectangle",Bounds=[300 150 1000 700])
end

%% Traffic Configuration

appDataRate = 1e3; % Application data rate in kilo bits per second (kbps) 
for ueIdx=1:numsUE 
    ulApps(ueIdx)=networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(UEs(ueIdx),ulApps(ueIdx)); 
    dlApps(ueIdx) = networkTrafficOnOff(GeneratePacket=true, OnTime=inf, OffTime=0, DataRate=appDataRate);
    addTrafficSource(gNBs(UEs(ueIdx).GNBNodeID),dlApps(ueIdx),DestinationNode=UEs(ueIdx));
end



%% network topology
networkVisualizer = helperNetworkVisualizer(SampleRate=100); % Sample rate indicates the visualization refresh rate in Hertz
addNodes(networkVisualizer,gNBs);
addNodes(networkVisualizer,UEs);

showBoundaries(networkVisualizer,gNBPositions,200);%圆形


%% 移动性管理
 h1=handoverManager(UEs(1),gNBs, networkSimulator,ulApps(1),dlApps(1),'train');
% h2=handoverManager(UEs(2),gNBs, networkSimulator,ulApps(2),dlApps(2),'train');


%% run
run(networkSimulator,1);








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


