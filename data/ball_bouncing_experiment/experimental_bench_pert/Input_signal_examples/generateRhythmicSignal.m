function [nsig, sig] = generateRhythmicSignal(dt, t_max, frequency, varargin)
%      
	while ~isempty(varargin)
        switch varargin{1}
            case 'TimeVariantMagnitude'
                TIME_VARIANT_MAGN = logical(varargin{2});
            case 'TimeVariantPhase'
                TIME_VARIANT_PHASE = logical(varargin{2});
            case 'VariantMagnitudeMaxFreq'
                fc_mag = double(varargin{2});
            case 'VariantPhaseMaxFreq'
                fc_ph = double(varargin{2});
            case 'FirstSineMagnitude'
                a1 = double(varargin{2});
            case 'SecondSineMagnitude'
                a2 = double(varargin{2});
            case 'GaussianNoise'
                isGaussianNoise = logical(varargin{2});
            otherwise
                error(['Unexpected option: ' varargin{1}])
        end
        varargin(1:2) = [];
	end
    if ~exist('TIME_VARIANT_MAGN', 'var')
        TIME_VARIANT_MAGN = false;
    end
    if ~exist('TIME_VARIANT_MAGN', 'var')
        TIME_VARIANT_MAGN = false;
    end
    if ~exist('fc_mag', 'var')
        fc_mag = 0.5; % Magnitude variations are low passed filtered at 0.5Hz
    end
    if ~exist('fc_ph', 'var')
        fc_ph = 0.25; % Phase variations are low passed filtered at 0.25Hz
    end
    if ~exist('a1', 'var')
        a1 = 1;       % first sine magnitude
    end
    if ~exist('a2', 'var')
        a2 = 0;%a1/5; % 2nd sine magnitude
    end
    if ~exist('isGaussianNoise', 'var')
        isGaussianNoise = false;
    end
    %% TODO: add those as parameters
    % signal
    alp = 10;                       % frequency multiplier for second sine
    phi = pi/2;                     % phase delay of second sine
    %
    t = 0:dt:t_max;                 % time vector
    t_under_samp = 0:10*dt:t_max;   % time

    if TIME_VARIANT_MAGN
        a1t = step(dsp.ColoredNoise('InverseFrequencyPower',2,'SamplesPerFrame',length(t)/10));
        a1t = 1 + interp1(t_under_samp, a1t/(max(a1t)-min(a1t)), t);
        % deal with last nan
        nan_idx = find(isnan(a1t));
        for nan_i = 1:length(nan_idx)
            a1t(nan_idx(nan_i)) = a1t(nan_idx(nan_i)-1);
        end
        [b,a] = butter(4,fc_mag/(1/(2*dt)),'low'); 
        a1t = a1.*filtfilt(b,a,a1t);
        a2t = a2.*filtfilt(b,a,a1t);
    else
        a1t = a1;
        a2t = a2;
    end

    if TIME_VARIANT_PHASE
        phit = step(dsp.ColoredNoise('InverseFrequencyPower',2,'SamplesPerFrame',length(t)/10));
        phit = phi.*interp1(t_under_samp, phit/(max(phit)-min(phit)), t);
        nan_idx = find(isnan(phit));
        for nan_i = 1:length(nan_idx)
            phit(nan_idx(nan_i)) = phit(nan_idx(nan_i)-1);
        end
        [b,a] = butter(4,fc_ph/(1/(2*dt)),'low'); 
        phit = filtfilt(b,a,phit);
    else
        phit = phi;
    end

    sig = a1t.*sin(2*pi*frequency.*t + phit) + a2t.*sin(alp*pi*frequency.*t + phit);

    % low freq noise
    %lf = f/20;
    %la = a1/4;
    %lsig = la*sin(2*pi*lf.*t);
    lsig = 0;
    if isGaussianNoise
        nsig = awgn(sig + lsig, 35);
    end

end

