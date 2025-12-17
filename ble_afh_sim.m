function ble_afh_sim()
% ble_afh_sim() - BLE AFH simulator
% run ble_afh_sim()

%% Simulation Parameters
rng(0); clearvars -except rng selAlg classAlg; close all;

if ~exist('selAlg', 'var')
    selAlg = 2;
end
if ~exist('classAlg', 'var')
    classAlg = 4; % valid: 0..4
end

sim.totalPackets = 20000; % increased to 20k recommended for prediction
sim.payloadBytes = 20; % bytes per logical payload (application data)
sim.maxRetransmit = 24; % max retransmissions (retries)
sim.connInterval_ms = 7.5; % connection interval (ms)
sim.phy_rate = 1e6; % physical layer bit rate (bps)
sim.sampleRate = 1e6; % sampling rate for GFSK baseband
sim.bt_gaussianBT = 0.5;
sim.modIndex = 0.5;
sim.preambleBits = 8*1;
sim.accessAddrBits = 32;
sim.crcBits = 24;
sim.psdu_len_bytes = sim.payloadBytes;

%% Noise & Interference Parameters (tune these)
chan.noise_dBm = -90;
chan.signal_power_dBm = -40;
chan.interference_power_dBm = -40;
chan.awgn_enable = true;
chan.interference_enable = true;

%% Real-world style interferers (example, some periodic)
interferers = struct(...
    'centerCh', { 5,   18,  30,  12,  7,   25,  2,   32,  20,  14,  9,   28,  16,  4,   22,  11,  33,  0,  35,  19 }, ...
    'bwCh',     { 20,  20, 20,  3,   2,   4,   1,   3,   2,   2,   1,   3,   2,   1,   3,   1,   2,   1,   1,   2 }, ...
    'mean_dBm', { -8, -6, -7, -0, -10, -15, -20, -18, -12, -15, -5, -20, -28, -28, -16, -22, -10, -35, -20, -13 }, ...
    'onProb',   { 0.95, 0.9, 0.9, 0.25, 0.12, 0.35, 0.05, 0.2,  0.4,  0.18, 0.07, 0.22, 0.15, 0.06, 0.3,  0.1,  0.6,  0.02, 0.08, 0.25 }, ... 
    'duration_mean_ms', { 2000, 1500, 1800, 80,  40,  300,  15,  120,  200,  60,   20,   100,  50,   10,   250,  30,   400,  8,   70,   90 }, ...
    'isPeriodic', { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false }, ...
    'burst_on_ms', { 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, ...
    'burst_interval_ms', { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } ...
    );
offset = log(1 / numel(interferers));
%fprintf('offset: %.f4\n', offset)
for k = 1:numel(interferers)
    interferers(k).mean_dBm = interferers(k).mean_dBm + chan.interference_power_dBm + offset;
end

%% BLE channel / AFH parameters
ble.Ndata = 37;   % data channels 0..36
ble.hopIncrement = 67;
ble.channel_map = true(1, ble.Ndata);
ble.chan_quality_window = 50;
ble.bad_rate_threshold = 0.4;
ble.min_channels_for_use = 20;
ble.recovery_time_ms = 2000;
ble.chan_quality_window = 50;

% choose AFH selection and classification algorithms
ble.afh_selection_algorithm = selAlg;         % 1,2,3
ble.afh_classification_algorithm = classAlg;    % 0..6 (5 & 6 predictive)

% selection algorithm-2 params
ble.accessAddress = uint32(hex2dec('8E89BED6'));
ble.channelIdentifier = uint16(0);
ble.eventCounter = 0;

% ---------- predictive algorithm parameters ----------
ble.pred_history_ms = 5000;   % how much past (ms) to use for prediction
ble.pred_bin_ms = 10;         % bin size for time-series (ms)
ble.pred_min_bins = 50;       % require at least this many bins to attempt FFT
ble.pred_amp_thresh = 6;      % dominant peak must be >= this * mean amplitude
ble.pred_lookahead_ms = 50;   % if next predicted peak within this, pre-mark bad
ble.pred_hold_ms = ble.recovery_time_ms; % hold after predicted interference
ble.pred_max_period_ms = 5000; % ignore periods longer than this
ble.pred_min_period_ms = 10;   % ignore periods shorter than this

%% Derived
bitsPerPacket = sim.preambleBits + sim.accessAddrBits + 8*sim.psdu_len_bytes + sim.crcBits;
packetTxTime_ms = (bitsPerPacket / sim.phy_rate) * 1e3;
dbm2mW = @(d) 10.^((d-30)/10);
signal_power_mW = dbm2mW(chan.signal_power_dBm);
noise_floor_mW = dbm2mW(chan.noise_dBm);

%% Initialize channel stats and interferer scheduling
chan.attempts = zeros(1, ble.Ndata);
chan.failures = zeros(1, ble.Ndata);
chan.lastBadTime_ms = -inf(1, ble.Ndata);
chan.isBad = ~ble.channel_map;

% per-channel energy history (dBm) and timestamps for predictive case 6
chan.energy_history = cell(1, ble.Ndata);
chan.energy_time = cell(1, ble.Ndata);
chan.energy_bad_count = zeros(1, ble.Ndata);

% per-channel failure time history for predictive case 5
chan.fail_time = cell(1, ble.Ndata);

% arrays to control periodic interferer bursts
interfererActive = false(1,numel(interferers));
nextInterfererEnd_ms = zeros(1,numel(interferers));
interfererInBurst = false(1,numel(interferers));
interfererNextBurstTransition_ms = zeros(1,numel(interferers));

%% Results
results.packetSent = 0;
results.packetSuccess = 0;
results.latencies_ms = [];
results.retransmissions = zeros(sim.totalPackets,1);
results.totalRetransmissions = 0;

%% Timeline / diagnostics storage
maxEvents = sim.totalPackets * (sim.maxRetransmit + 1);
chanStateTimeline = false(ble.Ndata, maxEvents);
eventChannel = zeros(1, maxEvents);
eventSuccess = false(1, maxEvents);
eventTime_ms = zeros(1, maxEvents);
eventIdx = 0;

%% Main loop
hopCounter = 0;
disconnections = 0;
simClock_ms = 0;
lastUsedChannel = 1;
for pktIdx = 1:sim.totalPackets
    appBits = randi([0 1], 1, 8*sim.psdu_len_bytes);
    attempts = 0;
    success = false;
    startTime_ms = simClock_ms;
    while ~success% && attempts <= sim.maxRetransmit
        attempts = attempts + 1;
        hopCounter = hopCounter + 1;
        if mod(attempts,sim.maxRetransmit)==0
            disconnections = disconnections + 1;
        end
        % ===== Update interferer sessions & periodic bursts =====
        for ii = 1:numel(interferers)
            if ~interfererActive(ii)
                if rand < interferers(ii).onProb * 0.01
                    interfererActive(ii) = true;
                    dur = max(1, round(interferers(ii).duration_mean_ms * (0.5 + rand)));
                    nextInterfererEnd_ms(ii) = simClock_ms + dur;
                    if interferers(ii).isPeriodic
                        interfererInBurst(ii) = true;
                        interfererNextBurstTransition_ms(ii) = simClock_ms + interferers(ii).burst_on_ms;
                    else
                        interfererInBurst(ii) = true;
                        interfererNextBurstTransition_ms(ii) = nextInterfererEnd_ms(ii);
                    end
                end
            else
                if simClock_ms >= nextInterfererEnd_ms(ii)
                    interfererActive(ii) = false;
                    interfererInBurst(ii) = false;
                    interfererNextBurstTransition_ms(ii) = 0;
                    nextInterfererEnd_ms(ii) = 0;
                else
                    if interferers(ii).isPeriodic && simClock_ms >= interfererNextBurstTransition_ms(ii)
                        if interfererInBurst(ii)
                            interfererInBurst(ii) = false;
                            interfererNextBurstTransition_ms(ii) = simClock_ms + interferers(ii).burst_interval_ms;
                        else
                            interfererInBurst(ii) = true;
                            interfererNextBurstTransition_ms(ii) = simClock_ms + interferers(ii).burst_on_ms;
                        end
                    end
                end
            end
        end

        %% select channel
        ble.eventCounter = ble.eventCounter + 1;
        ch = select_channel_AFH(hopCounter, lastUsedChannel, ble, chan);
        lastUsedChannel = ch;

        % update attempt counter
        chan.attempts(ch) = chan.attempts(ch) + 1;

        % generate packet bits (BER-based)
        pktBits = [randi([0 1],1, sim.preambleBits), randi([0 1],1, sim.accessAddrBits), appBits, randi([0 1],1, sim.crcBits)];

        % compute interference power for selected channel
        interfPower_mW = getInterferencePowerForChannel(ch, simClock_ms, interferers, interfererActive, nextInterfererEnd_ms, interfererInBurst);

        noise_mW = noise_floor_mW;
        totalNoisePlusInterf_mW = noise_mW + interfPower_mW;
        totalNoisePlusInterf_dBm = 10*log10(max(totalNoisePlusInterf_mW,1e-16)) + 30; % mW->dBm

        % push sample & timestamp into energy history
        max_energy_history = 2000; % keep long history for prediction
        h = chan.energy_history{ch};
        ht = chan.energy_time{ch};
        h(end+1) = totalNoisePlusInterf_dBm;
        ht(end+1) = simClock_ms;
        if numel(h) > max_energy_history
            h = h(end-max_energy_history+1:end);
            ht = ht(end-max_energy_history+1:end);
        end
        chan.energy_history{ch} = h;
        chan.energy_time{ch} = ht;

        % BER approx
        SNR_linear = signal_power_mW / max(1e-12, totalNoisePlusInterf_mW);
        EbN0 = SNR_linear * (sim.sampleRate / sim.phy_rate);
        ber = 0.5 * erfc( sqrt(EbN0/2) );
        ber = min(max(ber, 1e-12), 0.5);
        pktLenBits = length(pktBits);
        pErr = 1 - (1 - ber) ^ pktLenBits;

        % log event
        eventIdx = eventIdx + 1;
        chanStateTimeline(:, eventIdx) = ~chan.isBad(:);
        eventChannel(eventIdx) = ch;
        eventTime_ms(eventIdx) = simClock_ms;

        if rand > pErr
            success = true;
            eventSuccess(eventIdx) = true;
            results.packetSuccess = results.packetSuccess + 1;
            % optionally record success times if needed for predictive models
        else
            success = false;
            eventSuccess(eventIdx) = false;
            chan.failures(ch) = chan.failures(ch) + 1;
            chan.lastBadTime_ms(ch) = simClock_ms;
            % store failure timestamp for predictive case 5
            chan.fail_time{ch}(end+1) = simClock_ms;
            if numel(chan.fail_time{ch}) > 5000
                chan.fail_time{ch} = chan.fail_time{ch}(end-5000+1:end);
            end
        end

        % AFH classification (0..4)
        switch ble.afh_classification_algorithm
            case 0
                % baseline: do nothing

            case 1 % fixed-window failure rate
                if mod(sum(chan.attempts), ble.chan_quality_window) == 0
                    frate = chan.failures ./ max(1, chan.attempts);
                    for cidx = 1:ble.Ndata
                        if frate(cidx) > ble.bad_rate_threshold && ~chan.isBad(cidx)
                            chan.isBad(cidx) = true;
                            chan.lastBadTime_ms(cidx) = simClock_ms;
                            %fprintf('Mark channel %d BAD at t=%.1f ms (fail rate %.2f)\n', cidx-1, simClock_ms, frate(cidx));
                        end
                        if chan.isBad(cidx)
                            if (simClock_ms - chan.lastBadTime_ms(cidx)) > ble.recovery_time_ms
                                chan.isBad(cidx) = false;
                                chan.attempts(cidx) = 0;
                                chan.failures(cidx) = 0;
                                %fprintf('Recovering channel %d at t=%.1f ms\n', cidx-1, simClock_ms);
                            end
                        end
                    end
                    % enforce minimum
                    goodMask = ~chan.isBad;
                    numGood = sum(goodMask);
                    if numGood < ble.min_channels_for_use
                        deficit = ble.min_channels_for_use - numGood;
                        [~, idx] = sort(frate, 'ascend');
                        ii = 1; reEnabled = 0;
                        while reEnabled < deficit && ii <= ble.Ndata
                            c = idx(ii);
                            if chan.isBad(c)
                                chan.isBad(c) = false;
                                %fprintf('Forcing channel %d GOOD to meet min_channels at t=%.1f ms\n', c-1, simClock_ms);
                                reEnabled = reEnabled + 1;
                            end
                            ii = ii + 1;
                        end
                    end
                end

            case 2 % EWMA failure fraction
                if ~isfield(ble,'ewma_alpha'), ble.ewma_alpha = 0.2; end
                if ~isfield(ble,'ewma_bad_threshold'), ble.ewma_bad_threshold = 0.5; end
                if ~isfield(ble,'ewma_recovery_streak'), ble.ewma_recovery_streak = 5; end
                if ~isfield(ble,'min_channels_for_use'), ble.min_channels_for_use = 20; end

                if ~isfield(chan,'badnessEWMA') || isempty(chan.badnessEWMA)
                    chan.badnessEWMA = zeros(1, ble.Ndata);
                end
                if ~isfield(chan,'goodStreak') || isempty(chan.goodStreak)
                    chan.goodStreak = zeros(1, ble.Ndata);
                end

                for cidx = 1:ble.Ndata
                    a = chan.attempts(cidx);
                    f = chan.failures(cidx);
                    if a > 0
                        obs = f / a;
                        chan.badnessEWMA(cidx) = (1-ble.ewma_alpha)*chan.badnessEWMA(cidx) + ble.ewma_alpha*obs;
                        chan.attempts(cidx) = 0;
                        chan.failures(cidx) = 0;
                    else
                        obs = 0;
                    end

                    if chan.badnessEWMA(cidx) > ble.ewma_bad_threshold && ~chan.isBad(cidx)
                        chan.isBad(cidx) = true;
                        chan.lastBadTime_ms(cidx) = simClock_ms;
                        %fprintf('EWMA: Mark channel %d BAD at t=%.1f ms (ewma=%.2f)\n', cidx-1, simClock_ms, chan.badnessEWMA(cidx));
                    end

                    if chan.isBad(cidx)
                        if obs < 0.2
                            chan.goodStreak(cidx) = chan.goodStreak(cidx) + 1;
                        else
                            chan.goodStreak(cidx) = 0;
                        end
                        if chan.goodStreak(cidx) >= ble.ewma_recovery_streak && (simClock_ms - chan.lastBadTime_ms(cidx)) > ble.recovery_time_ms
                            chan.isBad(cidx) = false;
                            chan.badnessEWMA(cidx) = chan.badnessEWMA(cidx) * 0.5;
                            chan.goodStreak(cidx) = 0;
                            %fprintf('EWMA: Recovering channel %d at t=%.1f ms\n', cidx-1, simClock_ms);
                        end
                    end
                end

                if sum(~chan.isBad) < ble.min_channels_for_use
                    [~, idx] = sort(chan.badnessEWMA,'ascend');
                    needed = ble.min_channels_for_use - sum(~chan.isBad);
                    for ii = 1:needed
                        chan.isBad(idx(ii)) = false;
                    end
                    %fprintf('EWMA: Enforced min channels -> enabled %d best channels\n', needed);
                end

            case 3 % ENERGY / RSSI ED
                if ~isfield(ble,'energy_threshold_dBm'), ble.energy_threshold_dBm = -60; end
                if ~isfield(ble,'energy_history_len'), ble.energy_history_len = 8; end
                if ~isfield(ble,'energy_recovery_count'), ble.energy_recovery_count = 3; end
                if ~isfield(ble,'min_channels_for_use'), ble.min_channels_for_use = 20; end
                if ~isfield(ble,'energy_hysteresis_dB'), ble.energy_hysteresis_dB = 6; end

                for cidx = 1:ble.Ndata
                    samples = chan.energy_history{cidx};
                    if isempty(samples)
                        med = -Inf;
                    else
                        n = min(numel(samples), ble.energy_history_len);
                        med = median(samples(end-n+1:end));
                    end

                    if med > ble.energy_threshold_dBm && ~chan.isBad(cidx)
                        chan.isBad(cidx) = true;
                        chan.lastBadTime_ms(cidx) = simClock_ms;
                        chan.energy_bad_count(cidx) = 0;
                        %fprintf('ENERGY: Mark channel %d BAD at t=%.1f ms (med_dBm=%.1f)\n', cidx-1, simClock_ms, med);
                    end

                    if chan.isBad(cidx)
                        if med <= (ble.energy_threshold_dBm - ble.energy_hysteresis_dB)
                            chan.energy_bad_count(cidx) = chan.energy_bad_count(cidx) + 1;
                        else
                            chan.energy_bad_count(cidx) = 0;
                        end
                        if chan.energy_bad_count(cidx) >= ble.energy_recovery_count && (simClock_ms - chan.lastBadTime_ms(cidx)) > ble.recovery_time_ms
                            chan.isBad(cidx) = false;
                            chan.energy_bad_count(cidx) = 0;
                            chan.attempts(cidx) = 0;
                            chan.failures(cidx) = 0;
                            %fprintf('ENERGY: Recovering channel %d at t=%.1f ms\n', cidx-1, simClock_ms);
                        end
                    end
                end

                if sum(~chan.isBad) < ble.min_channels_for_use
                    energies = zeros(1, ble.Ndata);
                    for cidx = 1:ble.Ndata
                        s = chan.energy_history{cidx};
                        if isempty(s)
                            energies(cidx) = Inf;
                        else
                            energies(cidx) = median(s(max(1,end-ble.energy_history_len+1):end));
                        end
                    end
                    [~, idx] = sort(energies,'ascend');
                    need = ble.min_channels_for_use - sum(~chan.isBad);
                    ii = 1; re = 0;
                    while re < need && ii <= ble.Ndata
                        c = idx(ii);
                        if chan.isBad(c)
                            chan.isBad(c) = false;
                            %fprintf('ENERGY: Forcing channel %d GOOD to meet min_channels at t=%.1f ms\n', c-1, simClock_ms);
                            re = re + 1;
                        end
                        ii = ii + 1;
                    end
                end

            case 4 % combined
                if ~isfield(ble,'chan_quality_window'), ble.chan_quality_window = 50; end
                if mod(sum(chan.attempts), ble.chan_quality_window) ~= 0
                    break;
                end
                if ~isfield(ble,'comb_alpha'), ble.comb_alpha = 0.25; end
                if ~isfield(ble,'comb_bad_threshold'), ble.comb_bad_threshold = 0.45; end
                if ~isfield(ble,'energy_weight'), ble.energy_weight = 0.5; end
                if ~isfield(ble,'energy_norm_lo'), ble.energy_norm_lo = -90; end
                if ~isfield(ble,'energy_norm_hi'), ble.energy_norm_hi = -20; end
                if ~isfield(ble,'energy_history_len'), ble.energy_history_len = 8; end

                if ~isfield(chan,'comb_score') || isempty(chan.comb_score)
                    chan.comb_score = zeros(1,ble.Ndata);
                end

                for cidx = 1:ble.Ndata
                    a = chan.attempts(cidx);
                    f = chan.failures(cidx);
                    frac = f / max(1, a);
                    s = chan.energy_history{cidx};
                    if isempty(s)
                        eNorm = 0.5;
                    else
                        n = min(length(s), ble.energy_history_len);
                        med = median(s(end-n+1:end));
                        eNorm = (med - ble.energy_norm_lo) / (ble.energy_norm_hi - ble.energy_norm_lo);
                        eNorm = min(max(eNorm,0),1);
                    end
                    observed = (1-ble.energy_weight)*frac + ble.energy_weight*eNorm;
                    chan.comb_score(cidx) = (1-ble.comb_alpha)*chan.comb_score(cidx) + ble.comb_alpha*observed;
                    if a > 0
                        chan.attempts(cidx) = 0;
                        chan.failures(cidx) = 0;
                    end
                    if chan.comb_score(cidx) > ble.comb_bad_threshold && ~chan.isBad(cidx)
                        chan.isBad(cidx) = true;
                        chan.lastBadTime_ms(cidx) = simClock_ms;
                        %fprintf('COMB: Mark ch %d BAD at t=%.1f ms (score=%.3f)\n', cidx-1, simClock_ms, chan.comb_score(cidx));
                    end
                    if chan.isBad(cidx) && (simClock_ms - chan.lastBadTime_ms(cidx)) > ble.recovery_time_ms
                        if chan.comb_score(cidx) < (ble.comb_bad_threshold * 0.6)
                            chan.isBad(cidx) = false;
                            chan.comb_score(cidx) = chan.comb_score(cidx) * 0.5;
                            %fprintf('COMB: Recovering ch %d at t=%.1f ms\n', cidx-1, simClock_ms);
                        end
                    end
                end
                if isfield(ble,'host_channel_map') && ~isempty(ble.host_channel_map)
                    mask = ble.host_channel_map;
                    chan.isBad(~mask) = true;
                end
                if sum(~chan.isBad) < ble.min_channels_for_use
                    [~, idx] = sort(chan.comb_score, 'ascend');
                    need = ble.min_channels_for_use - sum(~chan.isBad);
                    ii = 1; re = 0;
                    while re < need && ii <= ble.Ndata
                        c = idx(ii);
                        if chan.isBad(c)
                            chan.isBad(c) = false;
                            %fprintf('COMB: Forcing channel %d GOOD to meet min_channels at t=%.1f ms\n', c-1, simClock_ms);
                            re = re + 1;
                        end
                        ii = ii + 1;
                    end
                end

            case 5 % Predictive from packet failures, currently unused
                for cidx = 1:ble.Ndata
                    ftimes = chan.fail_time{cidx};
                    if isempty(ftimes)
                        continue;
                    end
                    [bins_t, series] = bin_time_series(ftimes, [], simClock_ms, ble.pred_history_ms, ble.pred_bin_ms);
                    if numel(series) < ble.pred_min_bins
                        continue; % not enough history
                    end
                    [period_ms, next_peak_ms, strength] = compute_dominant_period_and_next_peak(bins_t, series, simClock_ms, ble);
                    if ~isnan(period_ms) && strength >= ble.pred_amp_thresh
                        if (next_peak_ms - simClock_ms) <= ble.pred_lookahead_ms || (simClock_ms >= next_peak_ms - (period_ms*0.25) && simClock_ms <= next_peak_ms + (period_ms*0.25))
                            if ~chan.isBad(cidx)
                                chan.isBad(cidx) = true;
                                chan.lastBadTime_ms(cidx) = simClock_ms;
                                %fprintf('PRED-F: Mark ch %d BAD at t=%.1f ms (period=%.1f ms, next_peak_in=%.1f ms, str=%.2f)\n', cidx-1, simClock_ms, period_ms, next_peak_ms-simClock_ms, strength);
                            end
                        end
                        % Recovery: if prediction says no upcoming interference for a while
                        if chan.isBad(cidx)
                            time_till_next = next_peak_ms - simClock_ms;
                            if time_till_next > ble.pred_lookahead_ms + ble.pred_hold_ms
                                chan.isBad(cidx) = false;
                                %fprintf('PRED-F: Recovering ch %d at t=%.1f ms (pred says clear)\n', cidx-1, simClock_ms);
                            end
                        end
                    end
                end

                % enforce minimum channels
                if sum(~chan.isBad) < ble.min_channels_for_use
                    recent_fail_counts = zeros(1,ble.Ndata);
                    for cidx = 1:ble.Ndata
                        recent_fail_counts(cidx) = sum(chan.fail_time{cidx} >= (simClock_ms - ble.pred_history_ms));
                    end
                    [~, idx] = sort(recent_fail_counts, 'ascend');
                    need = ble.min_channels_for_use - sum(~chan.isBad);
                    ii = 1; re = 0;
                    while re < need && ii <= ble.Ndata
                        c = idx(ii);
                        if chan.isBad(c)
                            chan.isBad(c) = false;
                            re = re + 1;
                            %fprintf('PRED-F: Forcing channel %d GOOD to meet min_channels at t=%.1f ms\n', c-1, simClock_ms);
                        end
                        ii = ii + 1;
                    end
                end

            case 6 % Predictive from energy, currently unused
                for cidx = 1:ble.Ndata
                    etimes = chan.energy_time{cidx};
                    evalues = chan.energy_history{cidx};
                    if isempty(etimes) || numel(etimes) < 2
                        continue;
                    end
                    [bins_t, series] = bin_time_series(etimes, evalues, simClock_ms, ble.pred_history_ms, ble.pred_bin_ms);
                    if numel(series) < ble.pred_min_bins
                        continue;
                    end
                    [period_ms, next_peak_ms, strength] = compute_dominant_period_and_next_peak(bins_t, series, simClock_ms, ble);
                    if ~isnan(period_ms) && strength >= ble.pred_amp_thresh
                        if (next_peak_ms - simClock_ms) <= ble.pred_lookahead_ms || (simClock_ms >= next_peak_ms - (period_ms*0.25) && simClock_ms <= next_peak_ms + (period_ms*0.25))
                            if ~chan.isBad(cidx)
                                chan.isBad(cidx) = true;
                                chan.lastBadTime_ms(cidx) = simClock_ms;
                                %fprintf('PRED-E: Mark ch %d BAD at t=%.1f ms (period=%.1f ms, next_peak_in=%.1f ms, str=%.2f)\n', cidx-1, simClock_ms, period_ms, next_peak_ms-simClock_ms, strength);
                            end
                        end
                        if chan.isBad(cidx)
                            time_till_next = next_peak_ms - simClock_ms;
                            if time_till_next > ble.pred_lookahead_ms + ble.pred_hold_ms
                                chan.isBad(cidx) = false;
                                %fprintf('PRED-E: Recovering ch %d at t=%.1f ms (pred says clear)\n', cidx-1, simClock_ms);
                            end
                        end
                    end
                end

            otherwise
                % unknown case: nothing
        end

        simClock_ms = simClock_ms + sim.connInterval_ms;

    end % attempts loop

    % record per-packet stats
    results.packetSent = results.packetSent + 1;
    results.retransmissions(pktIdx) = attempts - 1;
    results.totalRetransmissions = results.totalRetransmissions + (attempts - 1);
    latency_ms = simClock_ms - startTime_ms;
    results.latencies_ms(end+1) = latency_ms;

    if mod(pktIdx, 200) == 0
        %fprintf('Packet %d/%d, success %d, time %.1f s\n', pktIdx, sim.totalPackets, results.packetSuccess, simClock_ms/1000);
    end
end % packets

% trim timeline arrays
chanStateTimeline = chanStateTimeline(:, 1:eventIdx);
eventChannel = eventChannel(1:eventIdx);
eventSuccess = eventSuccess(1:eventIdx);
eventTime_ms = eventTime_ms(1:eventIdx);

simTotalTime_ms = simClock_ms;

%% Metrics & Summary
totalSent = results.packetSent;
totalSuccess = results.packetSuccess;
packetLossRate = 1 - totalSuccess/totalSent;
totalPayloadBits = totalSuccess * sim.payloadBytes * 8;
throughput_bps = totalPayloadBits / (simTotalTime_ms/1000);
p95_latency = prctile(results.latencies_ms, 95);
p99_latency = prctile(results.latencies_ms, 99);
max_latency = max(results.latencies_ms);

% Map names
selNames = containers.Map({1,2,3},{'1 simple ','2 pseudo-random','3 custom (unused)'});
classNames = containers.Map(0:6, {...
    '0 none (baseline)', ...
    '1 fixed-window failure-rate', ...
    '2 EWMA failure fraction', ...
    '3 energy (RSSI/ED) based', ...
    '4 combined (EWMA+energy)', ...
    '5 predictive (packet failures)', ...
    '6 predictive (energy)'});
selName = 'unknown';
if isKey(selNames, ble.afh_selection_algorithm), selName = selNames(ble.afh_selection_algorithm); end
className = 'unknown';
if isKey(classNames, ble.afh_classification_algorithm), className = classNames(ble.afh_classification_algorithm); end

retransPerPacketRatio = results.totalRetransmissions / max(1,totalSent);

fprintf('\n--- Summary ---\n');
fprintf('AFH selection: %s\n', selName);
fprintf('AFH classification: %s\n', className);
fprintf('Packets: %d, Success: %d, Retransmissions: %d (ratio=%.3f), Disconnections: %d\n', ...
    totalSent, totalSuccess, results.totalRetransmissions, retransPerPacketRatio, disconnections);
fprintf('Throughput: %.1f bps, %.3f KBps\n', throughput_bps);
fprintf('Median latency: %.2f ms, p95: %.2f ms, p99: %.2f ms, max: %.2f ms\n', median(results.latencies_ms), p95_latency, p99_latency, max_latency);

%% Plots
% 1) Channels vs time heatmap with success/failure overlay
figure('Name','Channels vs Event','NumberTitle','off','Position',[100 100 1100 500],'WindowStyle','docked');
imagesc(1:eventIdx, 0:ble.Ndata-1, 1-chanStateTimeline);
colormap(flipud(gray)); xlabel('Event index'); ylabel('BLE data channel (0..36)');
title('Channel allowed (white) vs excluded (black) over events'); colorbar; caxis([0 1]);
hold on;
succIdx = find(eventSuccess);
failIdx = find(~eventSuccess);
scatter(succIdx, eventChannel(succIdx)-1, 8, 'g', 'filled');
scatter(failIdx, eventChannel(failIdx)-1, 12, 'r', 'x');
legend({'success','failure'}, 'Location','northoutside', 'Orientation','horizontal');
hold off;

% 2) Retransmissions per packet
figure('Name','Retransmissions per Packet','NumberTitle','off','Position',[150 150 600 400],'WindowStyle','docked');
maxR = sim.maxRetransmit;
counts = histcounts(results.retransmissions(1:totalSent), -0.5:1:(maxR+0.5));
bar(0:maxR, counts, 'BarWidth', 0.7);
set(gca,'YScale','log');
xlabel('Retransmissions'); ylabel('Count (log scale)');
title('Retransmissions per packet (log y-axis)'); grid on;

% 3) Latency CDF
figure('Name','Latency CDF','NumberTitle','off','WindowStyle','docked');
histogram(results.latencies_ms, 100, 'Normalization','cdf');
xlabel('Latency (ms)'); ylabel('CDF'); title('Latency CDF'); grid on;

% 4) Latency vs Percentile (log y-scale)
percentiles = 0:1:100;
latVals = prctile(results.latencies_ms, percentiles);
figure('Name','Latency vs Percentile','NumberTitle','off','WindowStyle','docked');
semilogy(percentiles, max(latVals,1e-6), '-o','MarkerSize',4);
xlabel('Percentile (%)'); ylabel('Latency (ms, log scale)');
title('Latency vs Percentile (log y-axis)'); grid on;

end % main function

%% helper functions

function ch = select_channel_AFH(counter, last_ch, ble_in, chan_in)
    switch ble_in.afh_selection_algorithm
        case 1
            ch = select_channel_alg1(counter, ble_in, chan_in);
        case 2
            ch = select_channel_alg2(counter, last_ch, ble_in, chan_in);
        case 3
            ch = custom_selector(counter, last_ch, ble_in, chan_in);
        otherwise
            ch = select_channel_alg1(counter, ble_in, chan_in);
    end
end

function goodList = getGoodChannels(ble_in, chan_in)
    goodList = find(~chan_in.isBad);
end

function ch = select_channel_alg1(counter, ble_in, chan_in)
    goodList = getGoodChannels(ble_in, chan_in);
    N = numel(goodList);
    if N == 0
        ch = mod(counter, ble_in.Ndata) + 1;
        return;
    end
    idx = mod(counter * ble_in.hopIncrement, N) + 1;
    ch = goodList(idx);
end

function ch = select_channel_alg2(counter, last_ch, ble_in, chan_in)
    goodList = getGoodChannels(ble_in, chan_in);
    N = numel(goodList);
    ble_in.channelIdentifier = bitxor(uint16(bitshift(ble_in.accessAddress, -16)), uint16(bitand(ble_in.accessAddress, uint32(65535))));
    temp = bitxor(uint16(counter), ble_in.channelIdentifier);
    for i = 1:3
        tempb = int2bit(temp,16);
        tempbb = zeros(1,16);
        tempbb(1:8) = tempb(8:-1:1);
        tempbb(9:16) = tempb(16:-1:9);
        temp = bit2int(tempbb,16);
        temp = uint16(mod(((17*uint32(temp))+uint32(ble_in.channelIdentifier)),2^16));
    end
    unmappedChannel = mod(uint16(bitxor(temp, ble_in.channelIdentifier)),37);
    if N == 0
        ch = double(unmappedChannel)+1;
        return;
    end
    remapIndex = mod(double(unmappedChannel), N) + 1;
    ch = goodList(remapIndex);
end

function ch = custom_selector(counter, last_ch, ble_in, chan_in) % currently unused, same as alg 1
    goodList = getGoodChannels(ble_in, chan_in);
    N = numel(goodList);
    if N == 0
        ch = mod(counter, ble_in.Ndata) + 1;
        return;
    end
    idx = mod(counter * ble_in.hopIncrement, N) + 1;
    ch = goodList(idx);
end

function currentInterfPower = getInterferencePowerForChannel(chIdx, time_ms, interferers, interfererActive, nextInterfererEnd_ms, interfererInBurst)
    dbm2mW_local = @(d) 10.^((d-30)/10);
    currentInterfPower = 0;
    for ii = 1:numel(interferers)
        if interfererActive(ii)
            if interferers(ii).isPeriodic && ~interfererInBurst(ii)
                continue;
            end
            center = interferers(ii).centerCh + 1;
            bw = interferers(ii).bwCh;
            dist = abs(chIdx - center);
            if dist <= bw/2
                weight = 1 - dist/(bw/2);
                p = dbm2mW_local(interferers(ii).mean_dBm) * weight;
                currentInterfPower = currentInterfPower + p;
            end
        end
    end
end

function [bins_t, series] = bin_time_series(times, values, now_ms, history_ms, bin_ms)
    t_start = max(0, now_ms - history_ms);
    edges = t_start:bin_ms:now_ms;
    if numel(edges) < 2
        bins_t = [];
        series = [];
        return;
    end
    bins_t = edges(1:end-1) + bin_ms/2;
    nb = numel(bins_t);

    if isempty(times)
        series = zeros(1, nb);
        return;
    end

    times = times(:)';
    if isempty(values)
        counts = histcounts(times, edges);
        series = double(counts);
        return;
    end

    values = values(:)';
    [~, binIdx] = histc(times, edges);

    series = nan(1, nb);
    for bi = 1:nb
        idx = find(binIdx == bi);
        if isempty(idx)
            series(bi) = NaN;
        else
            series(bi) = mean(values(idx));
        end
    end

    nanIdx = isnan(series);
    validIdx = ~nanIdx;
    nValid = sum(validIdx);

    if nValid == 0
        % no valid samples -> fill with zeros
        series(:) = 0;
    elseif nValid == 1
        % only one sample -> fill all bins with that single value
        singleVal = series(validIdx);
        series(:) = singleVal;
    else
        % >=2 valid samples -> interpolate nearest
        series(nanIdx) = interp1(bins_t(validIdx), series(validIdx), bins_t(nanIdx), 'nearest', 'extrap');
    end
end


function [period_ms, next_peak_ms, strength] = compute_dominant_period_and_next_peak(bins_t, series, now_ms, ble)
    period_ms = NaN; next_peak_ms = NaN; strength = 0;
    if isempty(series) || numel(series) < ble.pred_min_bins
        return;
    end
    y = double(series);
    y = y - mean(y);
    N = numel(y);
    w = hamming(N)';
    yw = y .* w;
    Y = fft(yw);
    P2 = abs(Y / N);
    P1 = P2(1:floor(N/2));
    freqs = (0:floor(N/2)-1) / (N * (ble.pred_bin_ms/1000));
    % ignore DC (index 1)
    P1(1) = 0;
    [peakAmp, idx] = max(P1);
    meanAmp = mean(P1(P1>0));
    if isempty(meanAmp) || meanAmp == 0
        meanAmp = eps;
    end
    strength = peakAmp / meanAmp;
    if strength < ble.pred_amp_thresh
        return;
    end
    freqHz = freqs(idx);
    if freqHz <= 0
        return;
    end
    period_s = 1 / freqHz;
    period_ms = period_s * 1000;
    if period_ms < ble.pred_min_period_ms || period_ms > ble.pred_max_period_ms
        period_ms = NaN; next_peak_ms = NaN; strength = 0;
        return;
    end
    t = bins_t;
    omega = 2*pi*freqHz;
    cosref = cos(omega * (t/1000));
    sinref = sin(omega * (t/1000));
    a = sum(y .* cosref);
    b = sum(y .* sinref);
    phase = atan2(b, a); % rad
    base_t = (-phase) * (1000/omega);
    k = floor((now_ms - base_t) / period_ms);
    last_peak_ms = base_t + k * period_ms;
    if last_peak_ms > now_ms
        last_peak_ms = last_peak_ms - period_ms;
    end
    next_peak_ms = last_peak_ms + period_ms;
end

% Helper: convert integer to little-endian bit vector (LSB index 1)
function b = int2bit(val, nbits)
    b = zeros(1, nbits);
    for kk = 1:nbits
        b(kk) = bitget(uint32(val), kk);
    end
end

% Helper: convert bit vector (LSB index 1) to integer
function val = bit2int(bvec, nbits)
    val = uint16(0);
    for kk = 1:nbits
        val = val + uint16(bvec(kk)) * uint16(2^(kk-1));
    end
end
