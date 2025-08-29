classdef helperNRPhyLogger < handle
    %helperNRPhyLogger Phy statistics logging object
    %   The class implements per slot/symbol logging mechanism
    %   of the physical layer metrics. It is used to log the statistics of a cell

    %   Copyright 2022-2024 The MathWorks, Inc.

    properties
        %NCellID Cell ID to which the logging belongs
        NCellID (1, 1) {mustBeInteger, mustBeInRange(NCellID, 0, 1007)} = 1;

        % NumUEs Count of UEs in a cell
        NumUEs

        % NumSlotsFrame Number of slots in a 10ms time frame
        NumSlotsFrame

        % RxStatsLog Slot-by-slot/symbol-by-symbol log of the Rx statistics
        RxStatsLog

        % ColumnIndexMap Mapping the column names of logs to respective column indices
        % It is a map object
        ColumnIndexMap

        % SchedulingType Type of scheduling (slot based or symbol based)
        % Value 0 means slot based and value 1 means symbol based. The
        % default value is 0
        SchedulingType (1, 1) {mustBeInteger, mustBeInRange(SchedulingType, 0, 1)} = 0;
    end

    properties (SetAccess = private)
        % UEIdList RNTIs of UEs in a cell as row vector
        UEIdList
    end

    properties(Access = private)
        % CurrSlot Current slot in the frame
        % It is incremented by 1 slot for every NumSym symbols
        CurrSlot = -1

        % CurrFrame Current frame
        % It is incremented by 1 frame for every NumSlotsFrame slots
        CurrFrame = -1

        % CurrSymbol Current symbol
        % It is updated for every call to logRxStats
        CurrSymbol = -1

        %UERxStats Downlink receiver side statistics for the current symbol
        % It is an N-by-2 array, where N is the number of UEs. First and second
        % columns of the array contains the number of erroneous packets received
        % and the total number of received packets for each UE respectively
        UERxStats

        %GNBRxStats Uplink receiver side statistics for the current symbol
        % It is an N-by-2 array, where N is the number of UEs. First and
        % second columns of the array contains the number of erroneous packets
        % received and the total number of received packets from each UE
        % respectively
        GNBRxStats

        %PrevCumulativeULBlkErr Cumulative uplink block error information returned in the last query
        % It is an array of size N-by-2 where N is the number of UEs,
        % columns 1 and 2 contains the number of erroneously received
        % packets and total received packets, respectively
        PrevCumulativeULBlkErr

        %PrevCumulativeDLBlkErr Cumulative downlink block error information returned in the last query
        % It is an array of size N-by-2 where N is the number of UEs,
        % columns 1 and 2 contains the number of erroneously received
        % packets and total received packets, respectively
        PrevCumulativeDLBlkErr

        %GNB gNB node object
        % It is a scalar and object of type nrGNB
        GNB

        %UEs UE node objects
        % It is an array of node objects of type nrUE
        UEs

        %LogIndexCounter Current log index
        LogIndexCounter = 0;
    end

    properties (Access = private, Constant, Hidden)
        % Constants related to downlink and uplink information. These
        % constants are used for indexing logs and identifying plots
        %DownlinkIdx Index for all downlink information
        DownlinkIdx = 1;
        %UplinkIdx Index for all uplink information
        UplinkIdx = 2;

        %NumSym Number of symbols in a slot
        NumSym = 14;
    end

    methods (Access = public)
        function obj = helperNRPhyLogger(numFramesSim, gNB, UEs)
            %helperNRPhyLogger Construct Phy logging object
            %
            % OBJ = helperNRPhyLogger(NUMFRAMESSIM, GNB, UEs) Create a Phy logging object
            % for logging the traces.
            %
            % NumFramesSim      - Simulation time in terms of number of 10 ms frames
            %
            % GNB - It is a scalar and object of type nrGNB
            %
            % UEs - It is an array of node objects of type nrUE. They must be connected
            %       to GNB

            networkSimulator = wirelessNetworkSimulator.getInstance();
            obj.GNB = gNB;
            obj.UEs = UEs;
            obj.NCellID = gNB.NCellID;
            schedulerConfig = gNB.MACEntity.Scheduler.SchedulerConfig;
            obj.SchedulingType = schedulerConfig.SchedulingType;
            obj.ColumnIndexMap = containers.Map('KeyType','char','ValueType','double');
            obj.NumUEs = numel(obj.UEs);
            obj.UEIdList = 1:obj.NumUEs;
            obj.NumSlotsFrame = (10 * gNB.SubcarrierSpacing) / 15e3; % Number of slots in a 10 ms frame

            % Rx stats
            % Each row represents the statistics of each slot
            obj.RxStatsLog = constructLogFormat(obj, numFramesSim);

            obj.UERxStats = zeros(obj.NumUEs, 2);
            obj.GNBRxStats = zeros(obj.NumUEs, 2);
            obj.PrevCumulativeULBlkErr = zeros(obj.NumUEs, 2);
            obj.PrevCumulativeDLBlkErr = zeros(obj.NumUEs, 2);

            % Register periodic logging event with network simulator
            phyLogPeriodicity = ((15e3/gNB.SubcarrierSpacing)/14) * 1e-3; % In seconds
            scheduleAction(networkSimulator, @obj.logCellPhyStats, [], phyLogPeriodicity/2, phyLogPeriodicity);
        end

        function [dlPhyMetrics, ulPhyMetrics] = getPhyMetrics(obj, firstSlot, lastSlot, rntiList)
            %getPhyMetrics Return the Phy metrics
            %
            % [DLMETRICS, ULMETRICS] = getPhyMetrics(OBJ, FIRSTSLOT,
            % LASTSLOT, RNTILIST) returns the Phy metrics of the UEs with
            % specified RNTIs for both uplink and downlink
            %
            % FIRSTSLOT - Represents the starting slot number for querying the metrics
            %
            % LASTSLOT  - Represents the ending slot for querying the metrics
            %
            % RNTILIST - Radio network temporary identifiers of an array of UEs
            %
            % ULPHYMETRICS - It contains Phy metrics in uplink direction
            %
            % DLPHYMETRICS - It contains Phy metrics in downlink direction
            %
            % ULPHYMETRICS and DLPHYMETRICS are structures with following properties
            %
            %   RNTI - Radio network temporary identifier of a UE
            %
            %   DecodeFailures - Total number of decode failures
            %
            %   TotalPackets - Total number of packets

            outputStruct = repmat(struct('RNTI',0,'TotalPackets',0,'DecodeFailures',0),[numel(rntiList) 2]);
            stepLogStartIdx = (firstSlot-1) * obj.NumSym + 1;
            stepLogEndIdx = lastSlot * obj.NumSym;
            columnMap = obj.ColumnIndexMap;
            metricsColumnIndex = [columnMap('Number of Decode Failures(DL)'),...
                columnMap('Number of Packets(DL)'); columnMap('Number of Decode Failures(UL)'),...
                columnMap('Number of Packets(UL)')];

            % Index at which UE's information is stored
            [~,ueIdxList] = ismember(rntiList, obj.UEIdList);
            for logIdx = 1:2
                rxStatsLogs = zeros(numel(rntiList), 2);
                for stepIdx = stepLogStartIdx:stepLogEndIdx
                    rxStatsLogs(:, 1) = rxStatsLogs(:, 1) + obj.RxStatsLog{stepIdx, metricsColumnIndex(logIdx, 1)}(ueIdxList);
                    rxStatsLogs(:, 2) = rxStatsLogs(:, 2) + obj.RxStatsLog{stepIdx, metricsColumnIndex(logIdx, 2)}(ueIdxList);
                end

                for ueIdx = 1:numel(rntiList)
                    outputStruct(ueIdx, logIdx).RNTI = rntiList(ueIdx);
                    outputStruct(ueIdx, logIdx).DecodeFailures = rxStatsLogs(ueIdx, 1);
                    outputStruct(ueIdx, logIdx).TotalPackets = rxStatsLogs(ueIdx, 2);
                end
            end

            dlPhyMetrics = outputStruct(:, obj.DownlinkIdx);
            ulPhyMetrics = outputStruct(:, obj.UplinkIdx);
        end

        function logCellPhyStats(obj, ~, ~)
            %logCellPhyStats Log the Phy layer statistics

            obj.LogIndexCounter = obj.LogIndexCounter + 1;
            % Read the DL Rx stats for each UE
            for ueIdx = 1:obj.NumUEs
                ueStats = [obj.UEs(ueIdx).PhyEntity.StatDecodeFailures obj.UEs(ueIdx).PhyEntity.StatReceivedPackets];
                obj.UERxStats(ueIdx, :) = ueStats - obj.PrevCumulativeDLBlkErr(ueIdx, :);
                obj.PrevCumulativeDLBlkErr(ueIdx, :) = ueStats;
            end
            % Read the UL Rx stats for each UE from gNB
            gnbStats = [obj.GNB.PhyEntity.StatDecodeFailures obj.GNB.PhyEntity.StatReceivedPackets];
            obj.GNBRxStats = gnbStats - obj.PrevCumulativeULBlkErr;
            obj.PrevCumulativeULBlkErr = gnbStats;
            % Log the UL and DL reception logs
            logRxStats(obj, obj.LogIndexCounter, obj.UERxStats, obj.GNBRxStats);
        end

        function logRxStats(obj, symbolNumSimulation, UERxStats, GNBRxStats)
            %logRxStats Log the reception statistics
            %
            % logRxStats(OBJ, SYMBOLNUMSIMULATION, UERxStats, GNBRxStats) Logs the
            % reception statistics
            %
            % SYMBOLNUMSIMULATION - Symbol number in the simulation
            %
            % UERxStats - Represents a N-by-2 array, where N is the number of UEs.
            % First and second columns of the array contains the number of erroneous
            % packets received and the total number of received packets for each UE
            %
            % GNBRxStats - Represents a N-by-2 array, where N is the number of UEs.
            % First and second columns of the array contains the number of erroneous
            % packets received and the total number of received packets from each UE

            if isempty(UERxStats) % Downlink Rx stats
                UERxStats = zeros(obj.NumUEs, 2);
            end
            if isempty(GNBRxStats) % Uplink Rx stats
                GNBRxStats = zeros(obj.NumUEs, 2);
            end

            columnMap = obj.ColumnIndexMap;
            % Calculate symbol number in slot (0-13), slot number in frame
            % (0-obj.NumSlotsFrame), frame number, and timestamp(in milliseconds) in the simulation.
            slotDuration = 10/obj.NumSlotsFrame;
            obj.CurrSymbol = mod(symbolNumSimulation - 1, obj.NumSym);
            obj.CurrSlot = mod(floor((symbolNumSimulation - 1)/obj.NumSym), obj.NumSlotsFrame);
            obj.CurrFrame = floor((symbolNumSimulation-1)/(obj.NumSym * obj.NumSlotsFrame));
            timestamp = obj.CurrFrame * 10 + (obj.CurrSlot * slotDuration) + (obj.CurrSymbol * (slotDuration / 14));

            logIndex = (obj.CurrFrame * obj.NumSlotsFrame * obj.NumSym) +  ...
                (obj.CurrSlot * obj.NumSym) + obj.CurrSymbol + 1;
            obj.RxStatsLog{logIndex, columnMap('Timestamp')} = timestamp;
            obj.RxStatsLog{logIndex, columnMap('Frame')} = obj.CurrFrame;
            obj.RxStatsLog{logIndex, columnMap('Slot')} = obj.CurrSlot;
            obj.RxStatsLog{logIndex, columnMap('Symbol')} = obj.CurrSymbol;

            % Number of erroneous packets in downlink
            obj.RxStatsLog{logIndex, columnMap('Number of Decode Failures(DL)')} = UERxStats(:, 1);
            % Number of packets in downlink
            obj.RxStatsLog{logIndex, columnMap('Number of Packets(DL)')} = UERxStats(:, 2);
            % Number of erroneous packets in uplink
            obj.RxStatsLog{logIndex, columnMap('Number of Decode Failures(UL)')} = GNBRxStats(:, 1);
            % Number of packets in uplink
            obj.RxStatsLog{logIndex, columnMap('Number of Packets(UL)')} = GNBRxStats(:, 2);
        end

        function receptionLogs = getReceptionLogs(obj)
            %getReceptionLogs Return the Phy reception logs
            %
            % RECEPTIONLOGS = getReceptionLogs(OBJ) Returns the Phy reception logs
            %
            % RECEPTIONLOGS - It is (N+1)-by-P cell, where N represents the number of
            % slots in the simulation and P represents the number of columns for
            % slot-based scheduling. For symbol-based scheduling, N represents the
            % number of symbols in the simulation. The first row of the logs contains
            % titles for the logs. Each row (excluding the first row) in the logs
            % represents a slot and contains the following information.
            %  Frame                           - Frame number.
            %  Slot                            - Slot number in the frame.
            %  Symbol number                   - Symbol number in the slot
            %  Number of Decode Failures(DL) - N-by-1 array, where N is the
            %                                    number of UEs. Each
            %                                    element contains the
            %                                    number of decode
            %                                    failures in the downlink
            %  Number of Packets(DL)           - N-by-1 array, where N is the
            %                                    number of UEs. Each
            %                                    element contains the
            %                                    number of packets in the downlink
            %  Number of Decode Failures(UL) - N-by-1 array, where N is the
            %                                    number of UEs. Each
            %                                    element contains the
            %                                    number of decode
            %                                    failures in the uplink
            %  Number of Packets(UL)           - N-by-1 array, where N is the
            %                                    number of UEs. Each
            %                                    element contains the
            %                                    number of packets in the uplink

            % Get keys of columns (i.e. column names) in sorted order of values (i.e. column indices)
            [~, idx] = sort(cell2mat(values(obj.ColumnIndexMap)));
            columnTitles = keys(obj.ColumnIndexMap);
            columnTitles = columnTitles(idx);

            % Most recent log index for the current simulation
            lastLogIndex = (obj.CurrFrame)*obj.NumSlotsFrame*obj.NumSym + (obj.CurrSlot+1)*obj.NumSym;
            totalULPackets = zeros(obj.NumUEs, 1);
            totalErrULPackets = zeros(obj.NumUEs, 1);
            totalDLPackets = zeros(obj.NumUEs, 1);
            totalErrDLPackets = zeros(obj.NumUEs, 1);
            columnMap = obj.ColumnIndexMap;

            % Calculate statistics for the entire simulation
            for idx = 1:lastLogIndex
                totalErrDLPackets = totalErrDLPackets + obj.RxStatsLog{idx, columnMap('Number of Decode Failures(DL)')};
                totalDLPackets = totalDLPackets + obj.RxStatsLog{idx, columnMap('Number of Packets(DL)')};
                totalErrULPackets = totalErrULPackets + obj.RxStatsLog{idx, columnMap('Number of Decode Failures(UL)')};
                totalULPackets = totalULPackets + obj.RxStatsLog{idx, columnMap('Number of Packets(UL)')};
            end
            receptionLogs = [columnTitles; obj.RxStatsLog(1:lastLogIndex , :)];
        end
    end

    methods(Access = private)
        function logFormat = constructLogFormat(obj, numFramesSim)
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

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} =  0; % Symbol number
            obj.ColumnIndexMap('Symbol') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = zeros(obj.NumUEs, 1); % Number of erroneous packets in the downlink direction
            obj.ColumnIndexMap('Number of Decode Failures(DL)') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = zeros(obj.NumUEs, 1); % Number of packets in the downlink direction
            obj.ColumnIndexMap('Number of Packets(DL)') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = zeros(obj.NumUEs, 1); % Number of erroneous packets in the uplink direction
            obj.ColumnIndexMap('Number of Decode Failures(UL)') = columnIndex;

            columnIndex = columnIndex + 1;
            logFormat{1, columnIndex} = zeros(obj.NumUEs, 1); % Number of packets in the uplink direction
            obj.ColumnIndexMap('Number of Packets(UL)') = columnIndex;

            % Initialize Rx stats logs for all the symbols in the simulation time
            numSlotsSim = numFramesSim * obj.NumSlotsFrame; % Simulation time in units of slot duration
            logFormat = repmat(logFormat(1,:), numSlotsSim*obj.NumSym, 1);
        end
    end
end