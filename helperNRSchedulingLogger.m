classdef helperNRSchedulingLogger < handle
    %helperNRSchedulingLogger Scheduler logging mechanism
    %   The class implements logging mechanism. The following types of
    %   informations is logged:
    %   - Logs of CQI values for UEs over the bandwidth
    %   - Logs of resource grid assignment to UEs
    %
    %   helperNRSchedulingLogger Name-Value pairs:
    %
    %   LinkDirection     - Flag to indicate the link direction associated to
    %                       the plots to visualize

    %   Copyright 2022-2024 The MathWorks, Inc.

    properties
        %NCellID Cell ID to which the logging and visualization object belongs
        NCellID (1, 1) {mustBeInteger, mustBeInRange(NCellID, 0, 1007)} = 1;

        %NumUEs Count of UEs
        NumUEs

        %NumHARQ Number of HARQ processes
        % The default value is 16 HARQ processes
        NumHARQ (1, 1) {mustBeInteger, mustBeInRange(NumHARQ, 1, 16)} = 16;

        %NumFrames Number of frames in simulation
        NumFrames

        %SchedulingType Type of scheduling (slot based or symbol based)
        % Value 0 means slot based and value 1 means symbol based. The
        % default value is 0
        SchedulingType (1, 1) {mustBeMember(SchedulingType, [0, 1])} = 0;

        %DuplexMode Duplexing mode
        % Frequency division duplexing (FDD) or time division duplexing (TDD)
        % Value 0 means FDD and 1 means TDD. The default value is 0
        DuplexMode (1, 1) {mustBeMember(DuplexMode, [0, 1])} = 0;

        %ResourceAllocationType Type for Resource allocation type (RAT)
        % Value 0 means RAT-0 and value 1 means RAT-1. The default value is 1
        ResourceAllocationType (1, 1) {mustBeInteger, mustBeInRange(ResourceAllocationType, 0, 1)} = 1;

        %ColumnIndexMap Mapping the column names of logs to respective column indices
        % It is a map object
        ColumnIndexMap

        %GrantColumnIndexMap Mapping the column names of scheduling logs to respective column indices
        % It is a map object
        GrantLogsColumnIndexMap

        %NumRBs Number of resource blocks
        % A vector of two elements and represents the number of PDSCH and
        % PUSCH RBs respectively
        NumRBs = zeros(2, 1);

        %Bandwidth Carrier bandwidth
        % A vector of two elements and represents the downlink and uplink
        % bandwidth respectively
        Bandwidth

        %RBGSizeConfig Type of RBG table to use
        % Flag used in determining the RBGsize. Value 1 represents
        % (configuration-1 RBG table) or 2 represents (configuration-2 RBG
        % table) as defined in 3GPP TS 38.214 Section 5.1.2.2.1. The
        % default value is 1
        RBGSizeConfig = 1;

        %SchedulingLog Symbol-by-symbol log of the simulation
        % In FDD mode first element contains downlink scheduling
        % information and second element contains uplink scheduling
        % information. In TDD mode first element contains scheduling
        % information of both downlink and uplink
        SchedulingLog = cell(2, 1);

        %GrantLog Log of the scheduling grants
        % It also contains the parameters for scheduling decisions
        GrantLog

        %IsLogReplay Flag to decide the type of post-simulation visualization
        % whether to show plain replay of the resource assignment during
        % simulation or of the selected slot (or frame). During the
        % post-simulation visualization, setting the value to 1 just
        % replays the resource assignment of the simulation frame-by-frame
        % (or slot-by-slot). Setting value to 0 gives the option to select
        % a particular frame (or slot) to see the way resources are
        % assigned in the chosen frame (or slot)
        IsLogReplay

        %PeakDataRateDL Theoretical peak data rate in the downlink direction
        PeakDataRateDL

        %PeakDataRateUL Theoretical peak data rate in the uplink direction
        PeakDataRateUL

        %TraceIndexCounter Current log index
        TraceIndexCounter = 0;

        %LinkDirection  Indicates the link direction associated to the plots to visualize
        % It takes the values 0, 1, 2 and represent downlink, uplink, and both link
        % directions respectively. Default value is 2.
        LinkDirection = 2;
    end

    properties (GetAccess = public, SetAccess = private)
        % UEIdList RNTIs of UEs in a cell as row vector
        UEIdList

        %GNB gNB node object
        % It is a scalar and object of type nrGNB
        GNB

        %UEs UE node objects
        % It is an array of node objects of type nrUE
        UEs
    end

    properties (Constant)
        %NumSym Number of symbols in a slot
        NumSym = 14;

        %NominalRBGSizePerBW Nominal RBG size table
        % It is for the specified bandwidth in accordance with
        % 3GPP TS 38.214, Section 5.1.2.2.1
        NominalRBGSizePerBW = [
            36   2   4
            72   4   8
            144  8   16
            275  16  16
            ];

        % Duplexing mode related constants
        %FDDDuplexMode Frequency division duplexing mode
        FDDDuplexMode = 0;
        %TDDDuplexMode Time division duplexing mode
        TDDDuplexMode = 1;

        % Constants related to scheduling type
        %SymbolBased Symbol based scheduling
        SymbolBased = 1;
        %SlotBased Slot based scheduling
        SlotBased = 0;

        % Constants related to downlink and uplink information. These
        % constants are used for indexing logs and identifying plots
        %DownlinkIdx Index for all downlink information
        DownlinkIdx = 1;
        %UplinkIdx Index for all downlink information
        UplinkIdx = 2;
    end

    properties (Access = private)
        %NumSlotsFrame Number of slots in 10ms time frame
        NumSlotsFrame

        %CurrSlot Current slot in the frame
        CurrSlot

        %CurrFrame Current frame
        CurrFrame

        %CurrSymbol Current symbol in the slot
        CurrSymbol

        %NumLogs Number of logs to be created based on number of links
        NumLogs

        %SymbolInfo Information about how each symbol (UL/DL/Guard) is allocated
        SymbolInfo

        %SlotInfo Information about how each slot (UL/DL/Guard) is allocated
        SlotInfo

        %PlotIds IDs of the plots
        PlotIds

        %GrantCount Keeps track of count of grants sent
        GrantCount = 0

        %RBGSize Number of RBs in an RBG. First element represents RBG
        % size for PDSCHRBs and second element represents RBG size for
        % PUSCHRBS
        RBGSize = zeros(2, 1);

        %LogInterval Represents the log interval
        % It represents the difference (in terms of number of symbols) between
        % two consecutive rows which contains valid data in SchedulingLog
        % cell array
        LogInterval

        %StepSize Represents the granularity of logs
        StepSize

        %UEMetricsUL UE metrics for each slot in the UL direction
        % It is an array of size N-by-2 where N is the number of UEs in
        % each cell. Each column of the array contains the following
        % metrics: Transmitted bytes and pending buffer amount bytes.
        UEMetricsUL

        %UEMetricsDL UE metrics for each slot in the DL direction
        % It is an array of size N-by-2 where N is the number of UEs in
        % each cell. Each column of the array contains the following
        % metrics: Transmitted bytes, and pending buffer amount bytes.
        UEMetricsDL

        %PrevUEMetricsUL UE metrics returned in the UL direction for previous query
        % It is an array of size N-by-2 where N is the number of UEs in
        % each cell. Each column of the array contains the following
        % metrics: Transmitted bytes transmitted, and pending buffer amount
        % bytes.
        PrevUEMetricsUL

        %PrevUEMetricsDL UE metrics returned in the DL direction for previous query
        % It is an array of size N-by-2 where N is the number of UEs in
        % each cell. Each column of the array contains the following
        % metrics: Transmitted bytes, and pending buffer amount bytes.
        PrevUEMetricsDL

        %UplinkChannelQuality Current channel quality for the UEs in uplink
        % It is an array of size M-by-N where M and N represents the number
        % of UEs in each cell and the number of RBs respectively.
        UplinkChannelQuality

        %DownlinkChannelQuality Current channel quality for the UEs in downlink
        % It is an array of size M-by-N where M and N represents the number
        % of UEs in each cell and the number of RBs respectively.
        DownlinkChannelQuality

        %HARQProcessStatusUL HARQ process status for each UE in UL
        % It is an array of size M-by-N where M and N represents the number
        % of UEs and number of HARQ processes for each UE respectively. Each
        % element stores the last received new data indicator (NDI) values
        % in the uplink
        HARQProcessStatusUL

        %HARQProcessStatusDL HARQ process status for each UE in DL
        % It is an array of size M-by-N where M and N represents the number
        % of UEs and number of HARQ processes for each UE respectively. Each
        % element stores the last received new data indicator (NDI) values
        % in the downlink
        HARQProcessStatusDL

        %PeakDLSpectralEfficiency Theoretical peak spectral efficiency in
        % the downlink direction
        PeakDLSpectralEfficiency

        %PeakULSpectralEfficiency Theoretical peak spectral efficiency in
        % the uplink direction
        PeakULSpectralEfficiency

        %LogGranularity Granularity of logs
        % It indicates whether logging is done for each symbol or each slot
        % (1 slot = 14 symbols)
        LogGranularity = 14;

        %Events List of events registered. It contains list of periodic events
        % By default events are triggered after every slot boundary. This event
        % list contains events which depends on the traces or which
        % requires periodic trigger after each slot boundary.
        % It is an array of structures and contains following fields
        %    CallBackFn - Call back to invoke when triggering the event
        %    TimeToInvoke - Time at which event has to be invoked
        Events = [];

        %NetworkSimulator Simulator handle object
        NetworkSimulator = []
    end

    methods
        function obj = helperNRSchedulingLogger(numFramesSim, gNB, UEs, varargin)
            %helperNRSchedulingLogger Construct scheduling information logging object
            %
            % OBJ = helperNRSchedulingLogger(NUMFRAMESSIM, GNB, UEs) Create scheduling
            % information logging object.
            %
            % OBJ = helperNRSchedulingLogger(NUMFRAMESSIM, GNB, UEs, LINKDIR) Create
            % scheduling information logging object.
            %
            % NUMFRAMESSIM is simulation time in terms of number of 10 ms frames.
            %
            % GNB is an object of type nrGNB.
            %
            % UEs is an array of node objects of type nrUE. They must be connected to
            % the same GNB.

            % Initialize the properties
            for idx = 1:2:numel(varargin)
                obj.(varargin{idx}) = varargin{idx+1};
            end
            if isempty(obj.NetworkSimulator) && isempty(obj.IsLogReplay)
                obj.NetworkSimulator = wirelessNetworkSimulator.getInstance();
            end
            % Set number of frames in simulation
            obj.NumFrames = numFramesSim;

            % Trace logging
            obj.GNB = gNB;
            obj.UEs = UEs;
            obj.NumSlotsFrame = (10 * gNB.SubcarrierSpacing) / 15e3; % Number of slots in a 10 ms frame
            slotDuration = (10/obj.NumSlotsFrame)*1e-3;
            % Symbol duration for the given numerology
            symbolDuration = 1e-3/(14*(gNB.SubcarrierSpacing/15e3)); % Assuming normal cyclic prefix

            obj.NCellID = gNB.NCellID;
            obj.NumUEs = numel(obj.UEs);
            obj.UEIdList = 1:obj.NumUEs;
            obj.NumHARQ = gNB.NumHARQ;
            schedulerConfig = gNB.MACEntity.Scheduler.SchedulerConfig;
            obj.SchedulingType = schedulerConfig.SchedulingType;
            obj.ColumnIndexMap = containers.Map('KeyType','char','ValueType','double');
            obj.GrantLogsColumnIndexMap = containers.Map('KeyType','char','ValueType','double');

            % Maximum number of transmission layers for each UE in DL
            numLayersDL = min(gNB.NumTransmitAntennas*ones(numel(obj.NumUEs), 1),[UEs.NumReceiveAntennas]');
            % Maximum number of transmission layers for each UE in UL
            numLayersUL = min(gNB.NumReceiveAntennas*ones(numel(obj.NumUEs), 1), [UEs.NumTransmitAntennas]');

            % Set resource allocation type
            obj.ResourceAllocationType = schedulerConfig.ResourceAllocationType;

            % Verify Duplex mode and update the properties
            if strcmpi(gNB.DuplexMode, "TDD")
                obj.DuplexMode = obj.TDDDuplexMode;
            end
            if obj.DuplexMode == obj.TDDDuplexMode || obj.SchedulingType == obj.SymbolBased
                obj.LogGranularity = 1;
            end

            if strcmpi(gNB.DuplexMode, "TDD") % TDD
                obj.NumLogs = 1;
                dlulConfig = gNB.DLULConfigTDD;
                % Number of DL symbols in one DL-UL pattern
                numDLSymbols = dlulConfig.NumDLSlots*14 + dlulConfig.NumDLSymbols;
                % Number of UL symbols in one DL-UL pattern
                numULSymbols = dlulConfig.NumULSlots*14 + dlulConfig.NumULSymbols;
                % Number of symbols in one DL-UL pattern
                numSymbols = dlulConfig.DLULPeriodicity*(gNB.SubcarrierSpacing/15e3)*14;
                % Normalized scalar considering the downlink symbol
                % allocation in the frame structure
                scaleFactorDL = numDLSymbols/numSymbols;
                % Normalized scalar considering the uplink symbol allocation
                % in the frame structure
                scaleFactorUL = numULSymbols/numSymbols;
            else % FDD
                obj.NumLogs = 2;
                % Normalized scalars in the DL and UL directions are 1 for
                % FDD mode
                scaleFactorDL = 1;
                scaleFactorUL = 1;
            end

            obj.UEMetricsUL = zeros(obj.NumUEs, 2);
            obj.UEMetricsDL = zeros(obj.NumUEs, 2);
            obj.PrevUEMetricsUL = zeros(obj.NumUEs, 2);
            obj.PrevUEMetricsDL = zeros(obj.NumUEs, 2);

            % Store current UL and DL CQI values on the RBs for the UEs.
            obj.UplinkChannelQuality = cell(obj.NumUEs, 1);
            obj.DownlinkChannelQuality = cell(obj.NumUEs, 1);

            % Store the last received new data indicator (NDI) values for UL and DL HARQ
            % processes.
            obj.HARQProcessStatusUL = zeros(obj.NumUEs, obj.NumHARQ);
            obj.HARQProcessStatusDL = zeros(obj.NumUEs, obj.NumHARQ);
            obj.Bandwidth = [gNB.ChannelBandwidth gNB.ChannelBandwidth];

            % Calculate uplink and downlink peak data rates as per 3GPP TS
            % 37.910. The number of layers used for the peak DL data rate
            % calculation is taken as the average of maximum layers
            % possible for each UE. The maximum layers possible for each UE
            % is min(gNBTxAnts, ueRxAnts)
            % Determine the plots
            if obj.LinkDirection == 2
                % Downlink & Uplink
                obj.PlotIds = [obj.DownlinkIdx obj.UplinkIdx];
                % Average of the peak DL transmitted values for each UE
                obj.PeakDataRateDL = 1e-6*(sum(numLayersDL)/obj.NumUEs)*scaleFactorDL*8*(948/1024)*(obj.GNB.NumResourceBlocks*12)/symbolDuration;
                obj.PeakDataRateUL = 1e-6*(sum(numLayersUL)/obj.NumUEs)*scaleFactorUL*8*(948/1024)*(obj.GNB.NumResourceBlocks*12)/symbolDuration;
                % Calculate uplink and downlink peak spectral efficiency
                obj.PeakDLSpectralEfficiency = 1e6*obj.PeakDataRateDL/obj.Bandwidth(obj.DownlinkIdx);
                obj.PeakULSpectralEfficiency = 1e6*obj.PeakDataRateUL/obj.Bandwidth(obj.UplinkIdx);
            elseif obj.LinkDirection == 0 % Downlink
                obj.PlotIds = obj.DownlinkIdx;
                obj.PeakDataRateDL = 1e-6*(sum(numLayersDL)/obj.NumUEs)*scaleFactorDL*8*(948/1024)*(obj.GNB.NumResourceBlocks*12)/symbolDuration;
                % Calculate downlink peak spectral efficiency
                obj.PeakDLSpectralEfficiency = 1e6*obj.PeakDataRateDL/obj.Bandwidth(obj.DownlinkIdx);
            else % Uplink
                obj.PlotIds = obj.UplinkIdx;
                obj.PeakDataRateUL = 1e-6*(sum(numLayersUL)/obj.NumUEs)*scaleFactorUL*8*(948/1024)*(obj.GNB.NumResourceBlocks*12)/symbolDuration;
                % Calculate uplink peak spectral efficiency
                obj.PeakULSpectralEfficiency = 1e6*obj.PeakDataRateUL/obj.Bandwidth(obj.UplinkIdx);
            end

            % Initialize number of RBs, RBG size, CQI and metrics properties
            for idx = 1: numel(obj.PlotIds)
                logIdx = obj.PlotIds(idx);
                obj.NumRBs(logIdx) = gNB.NumResourceBlocks; % Number of RBs in DL/UL
                % Calculate the RBGSize
                rbgSizeIndex = min(find(obj.NumRBs(logIdx) <= obj.NominalRBGSizePerBW(:, 1), 1));
                if obj.RBGSizeConfig == 1
                    obj.RBGSize(logIdx) = obj.NominalRBGSizePerBW(rbgSizeIndex, 2);
                else
                    obj.RBGSize(logIdx) = obj.NominalRBGSizePerBW(rbgSizeIndex, 3);
                end
            end

            % Initialize the scheduling logs and resources grid related
            % properties
            for idx=1:min(obj.NumLogs, numel(obj.PlotIds))
                plotId = obj.PlotIds(idx);
                if obj.DuplexMode == obj.FDDDuplexMode
                    logIdx = plotId; % FDD
                else
                    logIdx = idx; % TDD
                end
                % Construct the log format
                obj.SchedulingLog{logIdx} = constructLogFormat(obj, logIdx);
            end

            % Construct the grant log format
            obj.GrantLog = constructGrantLogFormat(obj);

            if ~isempty(obj.IsLogReplay) && obj.SchedulingType == obj.SlotBased
                % Post simulation log visualization and slot based scheduling
                obj.StepSize = 1;
                obj.LogInterval = 1;
            else
                % Live visualization
                obj.LogInterval = obj.NumSym;
                if obj.SchedulingType % Symbol based scheduling
                    obj.StepSize = 1;
                else % Slot based scheduling
                    obj.StepSize = obj.NumSym;
                end
            end

            if isempty(obj.IsLogReplay)
                % Register periodic logging event with network simulator to log cell
                % scheduling statistics
                if strcmpi(gNB.DuplexMode, "TDD") || obj.SchedulingType == obj.SymbolBased
                    scheduleAction(obj.NetworkSimulator, @obj.logCellSchedulingStats, [], symbolDuration/2, symbolDuration);
                else
                    scheduleAction(obj.NetworkSimulator, @obj.logCellSchedulingStats, [], slotDuration/2, slotDuration);
                end

                % Create a listener object for the 'ScheduledResources' event. This helps
                % in logging the scheduling output of gNB
                addlistener(obj.UEs, 'PacketReceptionEnded', @(src, eventData) obj.logPHYMetrics(src, eventData));
                addlistener(obj.GNB, 'PacketReceptionEnded', @(src, eventData) obj.logPHYMetrics(src, eventData));
                addlistener(obj.GNB, 'ScheduledResources', @(src, eventData) obj.logSchedulingGrants(src, eventData));
            end
        end

        function logPHYMetrics(obj, ~, eventData)
            %logPHYMetrics Log the SINR and signal type values
            %
            % logPHYMetrics(OBJ, EVENTSOURCE, EVENTDATA) Logs
            % the PHY information based on the received event data
            %
            % EVENTSOURCE - Event source object
            %
            % EVENTDATA - Event data

            linkDir = obj.PlotIds;
            linkDir = linkDir - 1;
            if numel(obj.PlotIds) == 2
                linkDir = 2;
            end

            if obj.DuplexMode == obj.FDDDuplexMode
                % FDD Mode
                if linkDir ~= 1 && isa(eventData.Source, 'nrUE')
                    % if LinkDirection is for downlink or uplink and downlink
                    logPHYMetricsData(obj, eventData, obj.DownlinkIdx);
                end
                if linkDir ~= 0 && isa(eventData.Source, 'nrGNB')
                    % If LinkDirection is for uplink or uplink and downlink
                    logPHYMetricsData(obj, eventData, obj.UplinkIdx);
                end
            else % TDD mode
                logPHYMetricsData(obj, eventData, obj.DownlinkIdx);
            end
        end

        function logPHYMetricsData(obj, eventData, linkIndex)
            %logPHYMetricsData Assign the Scheduling log corresponding to the linkIndex

            data = eventData.Data;
            currFrame = data.TimingInfo(1);
            currSlot = data.TimingInfo(2);
            currSymbol = data.TimingInfo(3);
            symbolNumSimulation = (currFrame * obj.NumSlotsFrame + currSlot) * obj.NumSym + currSymbol;
            if ~isempty(data.SINR)
                if isa(eventData.Source, 'nrUE')
                    obj.SchedulingLog{linkIndex}{symbolNumSimulation+1, obj.ColumnIndexMap('SINR')}(eventData.Source.RNTI, 1) = data.SINR;
                else
                    obj.SchedulingLog{linkIndex}{symbolNumSimulation+1, obj.ColumnIndexMap('SINR')}(data.RNTI, 1) = data.SINR;
                end
                obj.SchedulingLog{linkIndex}{symbolNumSimulation+1, obj.ColumnIndexMap('Signal Type')}(data.RNTI, 1) = data.SignalType;
            end
        end

        function [dlMetrics, ulMetrics, cellMetrics] = getMACMetrics(obj, firstSlot, lastSlot, rntiList)
            %getMACMetrics Returns the MAC metrics
            %
            % [DLMETRICS, ULMETRICS] = getMACMetrics(OBJ, FIRSTSLOT,
            % LASTSLOT, RNTILIST) Returns the MAC metrics of the UE with
            % specified RNTI within the cell for both uplink and downlink direction
            %
            % FIRSTSLOT - Represents the starting slot number for
            % querying the metrics
            %
            % LASTSLOT -  Represents the ending slot for querying the metrics
            %
            % RNTILIST - Radio network temporary identifiers of the UEs
            %
            % ULMETRICS and DLMETRICS are array of structures with following properties
            %
            %   RNTI - Radio network temporary identifier of the UE
            %
            %   TransmittedBytes - Total number of bytes transmitted (newTx and reTx
            %                      combined)
            %
            %   BufferStatus - Current buffer status of the UE
            %
            %   AssignedRBCount - Number of resource blocks assigned to the UE
            %
            %   RBsScheduled - Total number resource blocks scheduled
            %
            % CELLMETRICS is an array structure with following properties and
            % contains cell wide metrics in downlink and uplink
            % respectively
            %
            %   DLTransmittedBytes - Total number of bytes transmitted (newTx and
            %   reTx combined) in downlink
            %
            %   DLRBsScheduled - Total number resource blocks scheduled in
            %   downlink
            %
            %   ULTransmittedBytes - Total number of bytes transmitted (newTx and reTx
            %   combined) in uplink
            %
            %   ULRBsScheduled - Total number resource blocks scheduled in uplink

            % Calculate the actual log start and end index
            stepLogStartIdx = (firstSlot-1) * obj.LogInterval + 1;
            stepLogEndIdx = lastSlot*obj.LogInterval;

            % Create structure for both DL and UL
            outStruct = struct('RNTI', 0, 'TransmittedBytes', 0, 'BufferStatus', 0, ...
                'AssignedRBCount', 0, 'RBsScheduled', 0);
            outputStruct = repmat(outStruct, [numel(rntiList) 2]);
            assignedRBsStep = zeros(obj.NumUEs, 2);
            macTxStep = zeros(obj.NumUEs, 2);
            bufferStatus = zeros(obj.NumUEs, 2);

            % Update the DL and UL metrics properties
            for idx = 1:min(obj.NumLogs, numel(obj.PlotIds))
                plotId = obj.PlotIds(idx);
                % Determine scheduling log index
                if obj.DuplexMode == obj.FDDDuplexMode
                    schedLogIdx = plotId;
                else
                    schedLogIdx = 1;
                end

                numULSyms = 0;
                numDLSyms = 0;

                % Read the information of each slot and update the metrics
                % properties
                for i = stepLogStartIdx:obj.StepSize:stepLogEndIdx
                    slotLog = obj.SchedulingLog{schedLogIdx}(i, :);
                    frequencyAssignment = slotLog{obj.ColumnIndexMap('Frequency Allocation')};
                    transmittedBytes = slotLog{obj.ColumnIndexMap('Transmitted Bytes')};
                    ueBufferStatus = slotLog{obj.ColumnIndexMap('Buffer Status of UEs')};
                    if(obj.DuplexMode == obj.TDDDuplexMode)
                        switch (slotLog{obj.ColumnIndexMap('Tx Type')})
                            case 'UL'
                                linkIdx = 2; % Uplink information index
                                numULSyms = numULSyms + 1;
                            case 'DL'
                                linkIdx = 1; % Downlink information index
                                numDLSyms = numDLSyms + 1;
                            otherwise
                                continue;
                        end
                    else
                        linkIdx = plotId;
                    end

                    % Calculate the RBs allocated to an UE based on
                    % resource allocation type (RAT)
                    for ueIdx = 1 : obj.NumUEs
                        if obj.ResourceAllocationType % RAT-1
                            numRBs = frequencyAssignment(ueIdx, 2);
                        else % RAT-0
                            numRBGs = sum(frequencyAssignment(ueIdx, :));
                            if frequencyAssignment(ueIdx, end) % If RBG is allocated
                                % If the last RBG of BWP is assigned, then it might not
                                % have same number of RBs as other RBG.
                                if(mod(obj.NumRBs(plotId), obj.RBGSize(plotId)) == 0)
                                    numRBs = numRBGs * obj.RBGSize(plotId);
                                else
                                    lastRBGSize = mod(obj.NumRBs(plotId), obj.RBGSize(plotId));
                                    numRBs = (numRBGs - 1) * obj.RBGSize(plotId) + lastRBGSize;
                                end
                            else
                                numRBs = numRBGs * obj.RBGSize(plotId);
                            end
                        end

                        assignedRBsStep(ueIdx, linkIdx) = assignedRBsStep(ueIdx, linkIdx) + numRBs;
                        macTxStep(ueIdx, linkIdx) = macTxStep(ueIdx, linkIdx) + transmittedBytes(ueIdx);
                        bufferStatus(ueIdx, linkIdx) = ueBufferStatus(ueIdx);
                    end
                end
            end

            % Extract required metrics of the UEs specified in rntiList
            for idx = 1:numel(obj.PlotIds)
                linkIdx = obj.PlotIds(idx);
                for listIdx = 1:numel(rntiList)
                    ueIdx = find(rntiList(listIdx) == obj.UEIdList);
                    outputStruct(listIdx, linkIdx).RNTI = rntiList(listIdx);
                    outputStruct(listIdx, linkIdx).AssignedRBCount = assignedRBsStep(ueIdx, linkIdx);
                    outputStruct(listIdx, linkIdx).TransmittedBytes = macTxStep(ueIdx, linkIdx);
                    outputStruct(listIdx, linkIdx).BufferStatus = bufferStatus(ueIdx, linkIdx);
                end
            end
            dlMetrics = outputStruct(:, obj.DownlinkIdx); % Downlink Info
            ulMetrics = outputStruct(:, obj.UplinkIdx); % Uplink Info
            % Cell wide metrics
            cellMetrics.DLTransmittedBytes = sum(macTxStep(:, obj.DownlinkIdx));
            cellMetrics.ULTransmittedBytes = sum(macTxStep(:, obj.UplinkIdx));
            cellMetrics.ULRBsScheduled = sum(assignedRBsStep(:, obj.UplinkIdx));
            cellMetrics.DLRBsScheduled = sum(assignedRBsStep(:, obj.DownlinkIdx));
        end

        function [resourceGrid, resourceGridReTxInfo, resourceGridHarqInfo, varargout] = getRBGridsInfo(obj, frameNumber, slotNumber)
            %plotRBGrids Return the resource grid information
            %
            % getRBGridsInfo(OBJ, FRAMENUMBER, SLOTNUMBER) Return the resource grid status
            %
            % FRAMENUMBER - Frame number
            %
            % SLOTNUMBER - Slot number
            %
            % RESOURCEGRID In FDD mode first element contains downlink
            % resource grid allocation status and second element contains uplink
            % resource grid allocation status. In TDD mode first element
            % contains resource grid allocation status for downlink and uplink.
            % Each element is a 2D resource grid of N-by-P matrix where 'N' is
            % the number of slot or symbols and 'P' is the number of RBs in the
            % bandwidth to store how UEs are assigned different time-frequency
            % resources.
            %
            % RESOURCEGRIDHARQINFO In FDD mode first element contains
            % downlink HARQ information and second element contains uplink
            % HARQ information. In TDD mode first element contains HARQ
            % information for downlink and uplink. Each element is a 2D
            % resource grid of N-by-P matrix where 'N' is the number of
            % slot or symbols and 'P' is the number of RBs in the bandwidth
            % to store the HARQ process
            %
            % RESOURCEGRIDRETXINFO First element contains transmission
            % status in downlink and second element contains transmission
            % status in uplink for FDD mode. In TDD mode first element
            % contains transmission status for both downlink and uplink.
            % Each element is a 2D resource grid of N-by-P matrix where 'N'
            % is the number of slot or symbols and 'P' is the number of RBs
            % in the bandwidth to store type:new-transmission or
            % retransmission.

            resourceGrid = cell(2, 1);
            resourceGridReTxInfo = cell(2, 1);
            resourceGridHarqInfo = cell(2, 1);
            if obj.SchedulingType % Symbol based scheduling
                frameLogStartIdx = (frameNumber * obj.NumSlotsFrame * obj.LogInterval) + (slotNumber * obj.LogInterval);
                frameLogEndIdx = frameLogStartIdx + obj.LogInterval;
            else % Slot based scheduling
                frameLogStartIdx = frameNumber * obj.NumSlotsFrame * obj.LogInterval;
                frameLogEndIdx = frameLogStartIdx + (obj.NumSlotsFrame * obj.LogInterval);
            end

            % Read the resource grid information from logs
            for idx = 1:min(obj.NumLogs, numel(obj.PlotIds))
                plotId = obj.PlotIds(idx);
                if obj.DuplexMode == obj.FDDDuplexMode
                    logIdx = obj.PlotIds(idx);
                else
                    logIdx = 1;
                    symSlotInfo = cell(14,1);
                end

                % Reset the resource grid status
                if obj.SchedulingType % Symbol based scheduling
                    numRows = obj.NumSym;
                else % Slot based scheduling
                    numRows = obj.NumSlotsFrame;
                end
                emptyGrid = zeros(numRows, obj.NumRBs(logIdx));
                resourceGrid{logIdx} = emptyGrid;
                resourceGridReTxInfo{logIdx} = emptyGrid;
                resourceGridHarqInfo{logIdx} = emptyGrid;

                slIdx = 0; % Counter to keep track of the number of symbols/slots to be plotted
                for i = frameLogStartIdx+1:obj.StepSize:frameLogEndIdx % For each symbol in the slot or each slot in the frame
                    slIdx = slIdx + 1;
                    slotLog = obj.SchedulingLog{logIdx}(i, :);
                    frequencyAssignment = slotLog{obj.ColumnIndexMap('Frequency Allocation')};
                    harqIds = slotLog{obj.ColumnIndexMap('HARQ Process')};
                    txType = slotLog{obj.ColumnIndexMap('Tx Type')};
                    % Symbol or slot information
                    if obj.DuplexMode == obj.TDDDuplexMode
                        symSlotInfo{slIdx} = slotLog{obj.ColumnIndexMap('Type')};
                    end
                    for j = 1 : obj.NumUEs % For each UE
                        if (strcmp(txType(j), 'newTx') || strcmp(txType(j), 'newTx-Start') || strcmp(txType(j), 'newTx-InProgress') || strcmp(txType(j), 'newTx-End'))
                            type = 1; % New transmission
                        else
                            type = 2; % Retransmission
                        end

                        % Updating the resource grid status and related
                        % information
                        if obj.ResourceAllocationType % RAT-1
                            frequencyAllocation = frequencyAssignment(j, :);
                            startRBIndex = frequencyAllocation(1);
                            numRB = frequencyAllocation(2);
                            resourceGrid{logIdx}(slIdx, startRBIndex+1 : startRBIndex+numRB) = j;
                            resourceGridReTxInfo{logIdx}(slIdx, startRBIndex+1 : startRBIndex+numRB) = type;
                            resourceGridHarqInfo{logIdx}(slIdx, startRBIndex+1 : startRBIndex+numRB) = harqIds(j);
                        else % RAT-0
                            RBGAllocationBitmap = frequencyAssignment(j, :);
                            for k=1:length(RBGAllocationBitmap) % For all RBGs
                                if(RBGAllocationBitmap(k) == 1)
                                    startRBIndex = (k - 1) * obj.RBGSize(plotId) + 1;
                                    endRBIndex = k * obj.RBGSize(plotId);
                                    if(k == length(RBGAllocationBitmap) && (mod(obj.NumRBs(plotId), obj.RBGSize(plotId)) ~=0))
                                        % If it is last RBG and it does not
                                        % have same number of RBs as other RBGs
                                        endRBIndex = (k - 1) * obj.RBGSize(plotId) + mod(obj.NumRBs(plotId), obj.RBGSize(plotId));
                                    end
                                    resourceGrid{logIdx}(slIdx, startRBIndex : endRBIndex) = j;
                                    resourceGridReTxInfo{logIdx}(slIdx, startRBIndex : endRBIndex) = type;
                                    resourceGridHarqInfo{logIdx}(slIdx, startRBIndex : endRBIndex) = harqIds(j);
                                end
                            end
                        end
                    end
                end
            end
            if obj.DuplexMode == obj.TDDDuplexMode
                varargout{1} = symSlotInfo;
            end
        end

        function dlCQIInfo = getCQIRBGridsInfo(obj, frameNumber, slotNumber)
            %getCQIRBGridsInfo Return channel quality information
            %
            % getCQIRBGridsInfo(OBJ, FRAMENUMBER, SLOTNUMBER) Return
            % resource grid channel quality information
            %
            % FRAMENUMBER - Frame number
            %
            % SLOTNUMBER - Slot number
            %
            % DLCQIINFO - Downlink channel quality information

            lwRowIndex = frameNumber * obj.NumSlotsFrame * obj.LogInterval;
            if obj.SchedulingType % Symbol based scheduling
                upRowIndex = lwRowIndex + (slotNumber + 1) * obj.LogInterval;
            else % Slot based scheduling
                upRowIndex = lwRowIndex + (slotNumber * obj.LogInterval) + 1;
            end

            plotId = obj.DownlinkIdx;
            if (obj.DuplexMode == obj.TDDDuplexMode) % TDD
                % Get the symbols types in the current frame
                symbolTypeInFrame = {obj.SchedulingLog{1}(lwRowIndex+1:upRowIndex, obj.ColumnIndexMap('Type'))};
                cqiInfo = zeros(obj.NumUEs, obj.NumRBs(plotId));
                % Get the DL symbol indices
                dlIdx = find(strcmp(symbolTypeInFrame{1}, 'DL'));
                if ~isempty(dlIdx)
                    cqiValues = zeros(obj.NumUEs, obj.NumRBs(1));
                    for ueIdx = 1:obj.NumUEs
                        cqiValues(ueIdx, :) = obj.SchedulingLog{1}{lwRowIndex + dlIdx(end), obj.ColumnIndexMap('Channel Quality')}{ueIdx}.CQI;
                    end
                    % Update downlink channel quality based on latest DL
                    % symbol/slot
                    cqiInfo = cqiValues;
                end
            elseif obj.PlotIds(1) == plotId
                cqiValues = zeros(obj.NumUEs, obj.NumRBs(plotId));
                for ueIdx = 1:obj.NumUEs
                    cqiValues(ueIdx, :) = obj.SchedulingLog{plotId}{upRowIndex, obj.ColumnIndexMap('Channel Quality')}{ueIdx}.CQI;
                end
                cqiInfo = cqiValues;
            end
            dlCQIInfo = cqiInfo;
        end

        function logCellSchedulingStats(obj, ~, ~)
            %logCellSchedulingStats Log the MAC layer statistics
            %
            % logCellSchedulingStats(OBJ, ~, ~) Logs the scheduling information based
            % on the received event data

            linkDir = obj.PlotIds;
            linkDir = linkDir - 1;
            if numel(obj.PlotIds) == 2
                linkDir = 2;
            end
            gNB = obj.GNB;
            ueNode = obj.UEs;
            obj.TraceIndexCounter = obj.TraceIndexCounter + 1;
            symbolNum = (obj.TraceIndexCounter - 1) * obj.LogGranularity + 1;
            statusInfo.BufferSize = sum(gNB.MACEntity.LCHBufferStatus,2);
            scheduler = gNB.MACEntity.Scheduler;
            statusInfo.ULChannelQuality = [scheduler.UEContext.CSIMeasurementUL];
            gNBStatistics = gNB.statistics("all");
            % Read Tx bytes sent for each UE
            obj.UEMetricsDL(:, 1) = [gNBStatistics.MAC.Destinations.TransmittedBytes]';
             obj.UEMetricsDL(:, 2) = statusInfo.BufferSize; % Read pending buffer (in bytes) on gNB, for all the UEs

            for ueIdx = 1:obj.NumUEs
                obj.HARQProcessStatusUL(ueIdx, :) = obj.UEs(ueIdx).MACEntity.HARQNDIUL;
                obj.HARQProcessStatusDL(ueIdx, :) = obj.UEs(ueIdx).MACEntity.HARQNDIDL;
                % Read the UL channel quality at gNB for each of the UEs for logging
                obj.UplinkChannelQuality{ueIdx} = statusInfo.ULChannelQuality(ueIdx); % 1 for UL
                % Read the DL channel quality at UEs for logging
                ueStatusInfo.DLChannelQuality = ueNode(ueIdx).MACEntity.CSIMeasurement;
                ueStatusInfo.BufferSize = sum(ueNode(ueIdx).MACEntity.LCGBufferStatus);
                ueStatistics = ueNode(ueIdx).statistics();
                obj.DownlinkChannelQuality{ueIdx} = ueStatusInfo.DLChannelQuality; % 0 for DL
                % Read transmitted bytes transmitted for the UE in the
                % current TTI for logging
                obj.UEMetricsUL(ueIdx, 1) = ueStatistics.MAC.TransmittedBytes;
                obj.UEMetricsUL(ueIdx, 2) = ueStatusInfo.BufferSize; % Read pending buffer (in bytes) on UE
            end

            if obj.DuplexMode == 1 % TDD
                % Get current symbol type: DL/UL/Guard
                numSlots = floor((symbolNum-1)/14);
                dlulSlotIndex = mod(numSlots, scheduler.CellConfig.NumDLULPatternSlots);
                symbolIndex = mod(symbolNum-1, 14);
                symbolType = scheduler.CellConfig.DLULSlotFormat(dlulSlotIndex+1, symbolIndex+1);
                if(symbolType == 0 && linkDir ~= 1) % DL
                    metrics = obj.UEMetricsDL;
                    metrics(:, 1) = metrics(:, 1) - obj.PrevUEMetricsDL(:, 1);
                    obj.PrevUEMetricsDL = obj.UEMetricsDL;
                    logScheduling(obj, symbolNum, metrics, obj.DownlinkChannelQuality, obj.HARQProcessStatusDL, symbolType);
                elseif(symbolType == 1 && linkDir ~= 0) % UL
                    metrics = obj.UEMetricsUL;
                    metrics(:, 1) = metrics(:, 1) - obj.PrevUEMetricsUL(:, 1);
                    obj.PrevUEMetricsUL = obj.UEMetricsUL;
                    logScheduling(obj, symbolNum, metrics, obj.UplinkChannelQuality, obj.HARQProcessStatusUL, symbolType);
                else % Guard
                    logScheduling(obj, symbolNum, zeros(obj.NumUEs, 3), zeros(obj.NumUEs, obj.NumRBs(1)), zeros(obj.NumUEs, 16), symbolType); % UL
                end
            else
                % Store the scheduling logs
                if linkDir ~= 1 %  DL
                    metrics = obj.UEMetricsDL;
                    metrics(:, 1) = metrics(:, 1) - obj.PrevUEMetricsDL(:, 1);
                    obj.PrevUEMetricsDL = obj.UEMetricsDL;
                    logScheduling(obj, symbolNum, metrics, obj.DownlinkChannelQuality, obj.HARQProcessStatusDL, 0); % DL
                end
                if linkDir ~= 0 % UL
                    metrics = obj.UEMetricsUL;
                    metrics(:, 1) = metrics(:, 1) - obj.PrevUEMetricsUL(:, 1);
                    obj.PrevUEMetricsUL = obj.UEMetricsUL;
                    logScheduling(obj, symbolNum, metrics, obj.UplinkChannelQuality, obj.HARQProcessStatusUL, 1); % UL
                end
            end

            % Invoke the dependent events after every slot
            if obj.SchedulingType
                if mod(symbolNum, 14) == 0 && symbolNum > 1
                    % Invoke the events at the last symbol of the slot
                    invokeDepEvents(obj, (symbolNum/14));
                end
            else
                % Invoke the events at the first symbol of the last slot in a frame
                if mod(symbolNum-1, 14) == 0 && symbolNum > 1
                    invokeDepEvents(obj, ((symbolNum-1)/14)+1);
                end
            end
        end

        function logScheduling(obj, symbolNumSimulation, UEMetrics, UECQIs, HarqProcessStatus, type)
            %logScheduling Log the scheduling operations
            %
            % logScheduling(OBJ, SYMBOLNUMSIMULATION,
            % UEMETRICS, UECQIS, HARQPROCESSSTATUS, RXRESULTUES, TYPE) Logs
            % the scheduling operations based on the input arguments
            %
            % SYMBOLNUMSIMULATION - Cumulative symbol number in the
            % simulation
            %
            % UEMETRICS - N-by-P matrix where N represents the number of
            % UEs and P represents the number of metrics collected.
            %
            % UECQIs - N-by-P matrix where N represents the number of
            % UEs and P represents the number of RBs.
            %
            % HARQPROCESSSTATUS - N-by-P matrix where N represents the number of
            % UEs and P represents the number of HARQ process.
            %
            % TYPE - Type will be based on scheduling type.
            %        - In slot based scheduling type takes two values.
            %          type = 0, represents the downlink and type = 1,
            %          represents uplink.
            %
            %        - In symbol based scheduling type takes three values.
            %          type = 0, represents the downlink, type = 1,
            %          represents uplink and type = 2 represents guard.

            % Determine the log index based on link type and duplex mode
            if obj.DuplexMode == obj.FDDDuplexMode
                if  type == 0
                    linkIndex = obj.DownlinkIdx; % Downlink log
                else
                    linkIndex = obj.UplinkIdx; % Uplink log
                end
            else
                % TDD
                linkIndex = 1;
            end

            % Calculate symbol number in slot (0-13), slot number in frame
            % (0-obj.NumSlotsFrame), and frame number in the simulation.
            slotDuration = 10/obj.NumSlotsFrame;
            obj.CurrSymbol = mod(symbolNumSimulation - 1, obj.NumSym);
            obj.CurrSlot = mod(floor((symbolNumSimulation - 1)/obj.NumSym), obj.NumSlotsFrame);
            obj.CurrFrame = floor((symbolNumSimulation-1)/(obj.NumSym * obj.NumSlotsFrame));
            timestamp = obj.CurrFrame * 10 + (obj.CurrSlot * slotDuration) + (obj.CurrSymbol * (slotDuration / 14));

            columnMap = obj.ColumnIndexMap;
            obj.SchedulingLog{linkIndex}{symbolNumSimulation, columnMap('Timestamp')} = timestamp;
            obj.SchedulingLog{linkIndex}{symbolNumSimulation, columnMap('Frame')} = obj.CurrFrame;
            obj.SchedulingLog{linkIndex}{symbolNumSimulation, columnMap('Slot')} = obj.CurrSlot;
            if obj.SchedulingType % Symbol based scheduling
                obj.SchedulingLog{linkIndex}{symbolNumSimulation, columnMap('Symbol Number')} = obj.CurrSymbol;
            end

            if(obj.DuplexMode == obj.TDDDuplexMode) % TDD
                % Log the type: DL/UL/Guard
                switch(type)
                    case 0
                        symbolTypeDesc = 'DL';
                    case 1
                        symbolTypeDesc = 'UL';
                    case 2
                        symbolTypeDesc = 'Guard';
                end
                obj.SchedulingLog{linkIndex}{symbolNumSimulation, obj.ColumnIndexMap('Type')} = symbolTypeDesc;
            end

            obj.SchedulingLog{linkIndex}{symbolNumSimulation, obj.ColumnIndexMap('Channel Quality')} = UECQIs;
            obj.SchedulingLog{linkIndex}{symbolNumSimulation, obj.ColumnIndexMap('HARQ NDI Status')} = HarqProcessStatus;
            obj.SchedulingLog{linkIndex}{symbolNumSimulation, obj.ColumnIndexMap('Transmitted Bytes')} = UEMetrics(:, 1); % Transmitted bytes sent by UEs
            obj.SchedulingLog{linkIndex}{symbolNumSimulation, obj.ColumnIndexMap('Buffer Status of UEs')} = UEMetrics(:, 2); % Current buffer status of UEs in bytes
        end

        function logSchedulingGrants(obj, ~, eventData)
            %logSchedulingGrants Log the scheduling grant information
            %
            % logScheduling(OBJ, EVENTSOURCE, EVENTDATA) Logs
            % the scheduling information based on the received event data
            %
            % EVENTSOURCE - Event source object
            %
            % EVENTDATA - Event data

            currFrame = eventData.Data.TimingInfo(1);
            currSlot = eventData.Data.TimingInfo(2);
            currSymbol = eventData.Data.TimingInfo(3);
            symbolNumSimulation = (currFrame * obj.NumSlotsFrame + currSlot) * obj.NumSym + currSymbol;
            grantList = {};
            columnMap = obj.ColumnIndexMap;
            grantLogsColumnIndexMap = obj.GrantLogsColumnIndexMap;
            grantList{1} = eventData.Data.DLGrants;
            grantList{2} = eventData.Data.ULGrants;

            for grantIdx = 1:2
                if(obj.DuplexMode == obj.TDDDuplexMode) % TDD
                    % Grant is received always in DL
                    linkIndex = 1;
                    obj.SchedulingLog{linkIndex}{symbolNumSimulation+1, obj.ColumnIndexMap('Type')} = 'DL';
                else
                    % Depending on PlotIds linkIndex will be set
                    if isscalar(obj.PlotIds)
                        % Either one of UL or DL logging is set
                        linkIndex = obj.PlotIds;
                    else
                        % If both UL & DL logging is required
                        linkIndex = grantIdx;
                    end
                end
                resourceAssignments = grantList{grantIdx};
                for j = 1:length(resourceAssignments)
                    % Fill logs w.r.t. each assignment
                    assignment = resourceAssignments(j);
                    % Calculate row number in the logs, for the Tx start
                    % symbol
                    logIndex = (currFrame * obj.NumSlotsFrame * obj.NumSym) +  ...
                        ((currSlot + assignment.SlotOffset) * obj.NumSym) + assignment.StartSymbol + 1;

                    allottedUE = assignment.RNTI;

                    % Fill the start Tx symbol logs
                    obj.SchedulingLog{linkIndex}{logIndex, columnMap('Frequency Allocation')}(allottedUE, :) = assignment.FrequencyAllocation;
                    obj.SchedulingLog{linkIndex}{logIndex, columnMap('MCS Index')}(allottedUE) = assignment.MCSIndex;
                    obj.SchedulingLog{linkIndex}{logIndex, columnMap('HARQ Process')}(allottedUE) = assignment.HARQID;
                    obj.SchedulingLog{linkIndex}{logIndex, columnMap('NDI')}(allottedUE) = assignment.NDI;
                    if obj.SchedulingType % Symbol based scheduling
                        obj.SchedulingLog{linkIndex}{logIndex, columnMap('Tx Type')}(allottedUE) = {[assignment.Type, '-Start']};
                        % Fill the logs from the symbol after Tx start, up to
                        % the symbol before Tx end
                        for k = 1:assignment.NumSymbols-2
                            obj.SchedulingLog{linkIndex}{logIndex + k, columnMap('Frequency Allocation')}(allottedUE, :) = assignment.FrequencyAllocation;
                            obj.SchedulingLog{linkIndex}{logIndex + k, columnMap('MCS Index')}(allottedUE) = assignment.MCSIndex;
                            obj.SchedulingLog{linkIndex}{logIndex + k, columnMap('HARQ Process')}(allottedUE) = assignment.HARQID;
                            obj.SchedulingLog{linkIndex}{logIndex + k, columnMap('NDI')}(allottedUE) = assignment.NDI;
                            obj.SchedulingLog{linkIndex}{logIndex + k, columnMap('Tx Type')}(allottedUE) = {[assignment.Type, '-InProgress']};
                        end

                        % Fill the last Tx symbol logs
                        obj.SchedulingLog{linkIndex}{logIndex + assignment.NumSymbols -1, columnMap('Frequency Allocation')}(allottedUE, :) = assignment.FrequencyAllocation;
                        obj.SchedulingLog{linkIndex}{logIndex + assignment.NumSymbols -1, columnMap('MCS Index')}(allottedUE) = assignment.MCSIndex;
                        obj.SchedulingLog{linkIndex}{logIndex + assignment.NumSymbols -1, columnMap('HARQ Process')}(allottedUE) = assignment.HARQID;
                        obj.SchedulingLog{linkIndex}{logIndex + assignment.NumSymbols -1, columnMap('NDI')}(allottedUE) = assignment.NDI;
                        obj.SchedulingLog{linkIndex}{logIndex + assignment.NumSymbols -1, columnMap('Tx Type')}(allottedUE) = {[assignment.Type, '-End']};
                    else % Slot based scheduling
                        obj.SchedulingLog{linkIndex}{logIndex, columnMap('Tx Type')}(allottedUE) = {assignment.Type};
                    end
                    obj.GrantCount  = obj.GrantCount + 1;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('RNTI')} = assignment.RNTI;
                    slotNumGrant = mod(currSlot + assignment.SlotOffset, obj.NumSlotsFrame);
                    if(currSlot + assignment.SlotOffset >= obj.NumSlotsFrame)
                        frameNumGrant = currFrame + 1; % Assignment is for a slot in next frame
                    else
                        frameNumGrant = currFrame;
                    end
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Frame')} = frameNumGrant;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Slot')} = slotNumGrant;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Frequency Allocation')} = assignment.FrequencyAllocation;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Start Symbol')} = assignment.StartSymbol;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Num Symbols')} = assignment.NumSymbols;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('MCS Index')} = assignment.MCSIndex;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('NumLayers')} = assignment.NumLayers;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('HARQ Process')} = assignment.HARQID;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('NDI')} = assignment.NDI;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('RV')} = assignment.RV;
                    obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Tx Type')} = assignment.Type;
                    if(isfield(assignment, 'FeedbackSlotOffset'))
                        % DL grant
                        obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Feedback Slot Offset (DL grants only)')} = assignment.FeedbackSlotOffset;
                        obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Grant Type')} = 'DL';
                    else
                        % UL Grant
                        obj.GrantLog{obj.GrantCount, grantLogsColumnIndexMap('Grant Type')} = 'UL';
                    end
                end
            end
        end

        function varargout = getSchedulingLogs(obj)
            %getSchedulingLogs Get the per-symbol logs of the whole simulation

            % Get keys of columns (i.e. column names) in sorted order of values (i.e. column indices)
            [~, idx] = sort(cell2mat(values(obj.ColumnIndexMap)));
            columnTitles = keys(obj.ColumnIndexMap);
            columnTitles = columnTitles(idx);
            varargout = cell(obj.NumLogs, 1);

            for logIdx = 1:obj.NumLogs
                if isempty(obj.SchedulingLog{logIdx})
                    continue;
                end
                if obj.SchedulingType
                    % Symbol based scheduling
                    finalLogIndex = (obj.CurrFrame)*obj.NumSlotsFrame*obj.NumSym + (obj.CurrSlot)*obj.NumSym + obj.CurrSymbol + 1;
                    obj.SchedulingLog{logIdx} = obj.SchedulingLog{logIdx}(1:finalLogIndex, :);
                    % For symbol based scheduling, keep 1 row per symbol
                    varargout{logIdx} = [columnTitles; obj.SchedulingLog{logIdx}(1:finalLogIndex, :)];
                else
                    % Slot based scheduling
                    finalLogIndex = (obj.CurrFrame)*obj.NumSlotsFrame*obj.NumSym + (obj.CurrSlot+1)*obj.NumSym;
                    obj.SchedulingLog{logIdx} = obj.SchedulingLog{logIdx}(1:finalLogIndex, :);
                    % For slot based scheduling: keep 1 row per slot and eliminate symbol number as a column title
                    varargout{logIdx} = [columnTitles; obj.SchedulingLog{logIdx}(1:obj.NumSym:finalLogIndex, :)];
                end
            end
        end

        function logs = getGrantLogs(obj)
            %getGrantLogs Get the scheduling assignment logs of the whole simulation

            % Get keys of columns (i.e. column names) in sorted order of values (i.e. column indices)
            [~, idx] = sort(cell2mat(values(obj.GrantLogsColumnIndexMap)));
            columnTitles = keys(obj.GrantLogsColumnIndexMap);
            columnTitles = columnTitles(idx);
            % Read valid rows
            obj.GrantLog = obj.GrantLog(1:obj.GrantCount, :);
            logs = [columnTitles; obj.GrantLog];
        end

        function [dlStats, ulStats] = getPerformanceIndicators(obj)
            %getPerformanceIndicators Outputs the data rate, spectral
            % efficiency values
            %
            % DLSTATS - 4-by-1 array containing the following statistics in
            %           the downlink direction: Theoretical peak data rate,
            %           achieved data rate, theoretical peak spectral
            %           efficiency, achieved spectral efficiency
            % ULSTATS - 4-by-1 array containing the following statistics in
            %           the uplink direction: Theoretical peak data rate,
            %           achieved data rate, theoretical peak spectral
            %           efficiency, achieved spectral efficiency

            if obj.DuplexMode == obj.FDDDuplexMode
                if ismember(obj.DownlinkIdx, obj.PlotIds)
                    totalDLTxBytes = sum(cell2mat(obj.SchedulingLog{obj.DownlinkIdx}(:,  obj.ColumnIndexMap('Transmitted Bytes'))));
                end
                if ismember(obj.UplinkIdx, obj.PlotIds)
                    totalULTxBytes = sum(cell2mat(obj.SchedulingLog{obj.UplinkIdx}(:,  obj.ColumnIndexMap('Transmitted Bytes'))));
                end
            else
                dlIdx = strcmp(obj.SchedulingLog{1}(:, obj.ColumnIndexMap('Type')), 'DL');
                totalDLTxBytes = sum(cell2mat(obj.SchedulingLog{1}(dlIdx,  obj.ColumnIndexMap('Transmitted Bytes'))));
                ulIdx = strcmp(obj.SchedulingLog{1}(:, obj.ColumnIndexMap('Type')), 'UL');
                totalULTxBytes = sum(cell2mat(obj.SchedulingLog{1}(ulIdx,  obj.ColumnIndexMap('Transmitted Bytes'))));
            end
            dlStats = zeros(4, 1);
            ulStats = zeros(4, 1);

            % Downlink stats
            if ismember(obj.DownlinkIdx, obj.PlotIds)
                dlStats(1, 1) = obj.PeakDataRateDL;
                dlStats(2, 1) = totalDLTxBytes * 8 ./ (obj.NumFrames * 0.01 * 1000 * 1000); % Mbps
                dlStats(3, 1) = obj.PeakDLSpectralEfficiency;
                dlStats(4, 1) = 1e6*dlStats(2, 1)/obj.Bandwidth(obj.DownlinkIdx);
            end
            % Uplink stats
            if ismember(obj.UplinkIdx, obj.PlotIds)
                ulStats(1, 1) = obj.PeakDataRateUL;
                ulStats(2, 1) = totalULTxBytes * 8 ./ (obj.NumFrames * 0.01 * 1000 * 1000); % Mbps
                ulStats(3, 1) = obj.PeakULSpectralEfficiency;
                ulStats(4, 1) = 1e6*ulStats(2, 1)/obj.Bandwidth(obj.UplinkIdx);
            end
        end

        function addDepEvent(obj, callbackFcn, numSlots)
            %addDepEvent Adds an event to the events list
            %
            % addDepEvent(obj, callbackFcn, numSlots) Adds an event to the
            % event list
            %
            % CALLBACKFCN - Handle of the function to be invoked
            %
            % NUMSLOTS - Periodicity at which function has to be invoked

            % Create event
            event = struct('CallbackFcn', callbackFcn, 'InvokePeriodicity', numSlots);
            obj.Events = [obj.Events  event];
        end
    end

    methods( Access = private)
        function invokeDepEvents(obj, slotNum)
            numEvents = length(obj.Events);
            for idx=1:numEvents
                event = obj.Events(idx);
                if isempty(event.InvokePeriodicity)
                    event.CallbackFcn(slotNum);
                else
                    invokePeriodicity = event.InvokePeriodicity;
                    if mod(slotNum, invokePeriodicity) == 0
                        event.CallbackFcn(slotNum);
                    end
                end
            end
        end

        function logFormat = constructLogFormat(obj, linkIdx)
            %constructLogFormat Construct log format

            columnIndex = 1;
            logFormat{1, columnIndex} = 0; % Timestamp (in milliseconds)
            obj.ColumnIndexMap('Timestamp') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = 0; % Frame number
            obj.ColumnIndexMap('Frame') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} =  0; % Slot number
            obj.ColumnIndexMap('Slot') = columnIndex;

            if(obj.SchedulingType == 1)
                % Symbol number column is only for symbol-based
                % scheduling
                columnIndex = columnIndex + 1;
                logFormat{1, columnIndex} =  0; % Symbol number
                obj.ColumnIndexMap('Symbol Number') = columnIndex;
            end
            if(obj.DuplexMode == obj.TDDDuplexMode)
                % Slot/symbol type as DL/UL/guard is only for TDD mode
                columnIndex = columnIndex + 1;
                logFormat{1, columnIndex} = 'Guard'; % Symbol type
                obj.ColumnIndexMap('Type') = columnIndex;
            end

            columnIndex = columnIndex + 1;
            if obj.ResourceAllocationType % RAT-1
                logFormat{1, columnIndex} = zeros (obj.NumUEs, 2); % RB allocation for UEs
            else  % RAT-0
                logFormat{1, columnIndex} = zeros(obj.NumUEs, ceil(obj.NumRBs(linkIdx) / obj.RBGSize(linkIdx))); % RBG allocation for UEs
            end
            obj.ColumnIndexMap('Frequency Allocation') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1*ones(obj.NumUEs, 1); % MCS for assignments
            obj.ColumnIndexMap('MCS Index') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1*ones(obj.NumUEs, 1); % HARQ IDs for assignments
            obj.ColumnIndexMap('HARQ Process') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1*ones(obj.NumUEs, 1); % NDI flag for assignments
            obj.ColumnIndexMap('NDI') = columnIndex;

            % Tx type of the assignments ('newTx' or 'reTx'), 'noTx' if there is no assignment
            txTypeUEs =  cell(obj.NumUEs, 1);
            txTypeUEs(:) = {'noTx'};
            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = txTypeUEs;
            obj.ColumnIndexMap('Tx Type') = columnIndex;

            columnIndex = columnIndex + 1;
            if linkIdx
                % Initialize CSI report format for DL
                csiReport = struct('RankIndicator', 1, 'PMISet', [], 'CQI', zeros(1, obj.NumRBs(linkIdx)), 'CSIResourceIndicator', [], 'L1RSRP', []);
            else
                % Initialize CSI report format for UL
                csiReport = struct('RankIndicator', 1, 'TPMI', [], 'MCSIndex', 0);
            end
            logFormat{1, columnIndex} = repmat({csiReport}, obj.NumUEs, 1); % Channel quality
            obj.ColumnIndexMap('Channel Quality') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = zeros(obj.NumUEs, obj.NumHARQ); % HARQ process status
            obj.ColumnIndexMap('HARQ NDI Status') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = zeros(obj.NumUEs, 1); % MAC bytes transmitted
            obj.ColumnIndexMap('Transmitted Bytes') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = zeros(obj.NumUEs, 1); % UEs' buffer status
            obj.ColumnIndexMap('Buffer Status of UEs') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -Inf(obj.NumUEs, 1); % SINR values
            obj.ColumnIndexMap('SINR') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = strings(obj.NumUEs, 1); % Received signal type
            obj.ColumnIndexMap('Signal Type') = columnIndex;

            % Initialize scheduling log for all the symbols in the
            % simulation time. The last time scheduler runs in the
            % simulation, it might assign resources for future slots which
            % are outside of simulation time. Storing those decisions too
            numSlotsSim = obj.NumFrames * obj.NumSlotsFrame; % Simulation time in units of slot duration
            logFormat = repmat(logFormat(1,:), (numSlotsSim + obj.NumSlotsFrame)*obj.NumSym , 1);
        end

        function logFormat = constructGrantLogFormat(obj)
            %constructGrantLogFormat Construct grant log format

            columnIndex = 1;
            logFormat{1, columnIndex} = -1; % UE's RNTI
            obj.GrantLogsColumnIndexMap('RNTI') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % Frame number
            obj.GrantLogsColumnIndexMap('Frame') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % Slot number
            obj.GrantLogsColumnIndexMap('Slot') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = {''}; % Type: UL or DL
            obj.GrantLogsColumnIndexMap('Grant Type') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = {''}; % Frequency allocation for UEs
            obj.GrantLogsColumnIndexMap('Frequency Allocation') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % Start Symbol
            obj.GrantLogsColumnIndexMap('Start Symbol') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % Num Symbols
            obj.GrantLogsColumnIndexMap('Num Symbols') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % MCS Value
            obj.GrantLogsColumnIndexMap('MCS Index') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % Number of layers
            obj.GrantLogsColumnIndexMap('NumLayers') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % HARQ IDs for assignments
            obj.GrantLogsColumnIndexMap('HARQ Process') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % NDI flag for assignments
            obj.GrantLogsColumnIndexMap('NDI') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = -1; % RV for assignments
            obj.GrantLogsColumnIndexMap('RV') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = {''}; % Tx type: new-Tx or re-Tx
            obj.GrantLogsColumnIndexMap('Tx Type') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = {'NA'}; % PDSCH feedback slot offset (Only applicable for DL grants)
            obj.GrantLogsColumnIndexMap('Feedback Slot Offset (DL grants only)') = columnIndex;

            % Initialize scheduling log for all the symbols in the
            % simulation time. The last time scheduler runs in the
            % simulation, it might assign resources for future slots which
            % are outside of simulation time. Storing those decisions too
            if obj.SchedulingType == 1
                maxRows = obj.NumFrames*obj.NumSlotsFrame*obj.NumUEs*(ceil(obj.NumSym/gNB.MACEntity.Scheduler.TTIGranularity));
            else
                maxRows = obj.NumFrames*obj.NumSlotsFrame*obj.NumUEs;
            end
            logFormat = repmat(logFormat(1,:), maxRows , 1);
        end

    end
end