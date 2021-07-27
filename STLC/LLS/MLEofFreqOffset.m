% Compensate Maximum Likelihood Estimation(MLE) of Frequency offset
clc; close all; clear all;

fftSize = 1024;
nbcp = 72;
nbcp0 = 16;
fOffset = fftSize-600/2;

Ndata = 5;
Ns = 2;
samplingRate = 15360000;

QPSKsym = [1+1j 1-1j -1+1j -1-1j]/sqrt(2);

% Switch option to 0 or 1
option_MLfreqOffsetEst = 1;
option_offsetComp = 0;

% preamble sequence pattern 
preamble_seq = [4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0];
preamble_seq_real = convertToReal(preamble_seq);

lowerCnt = floor(length(preamble_seq_real)/2)+1;
upperCnt = length(preamble_seq_real)-lowerCnt;

% preamble signal generation (freq domain signal generation)
preamble_seq_freq = zeros(1,fftSize);
preamble_seq_freq(fftSize- lowerCnt+1: fftSize) = preamble_seq_real(1:lowerCnt); 
preamble_seq_freq(1:upperCnt) = preamble_seq_real(length(preamble_seq_real) - upperCnt+1:length(preamble_seq_real));

% time domain signal
preamble_seqTime = ifft(preamble_seq_freq, fftSize);

x = convertToReal(load('./rxdata_MLEofFreqOffset.txt'));

figure(1); %signal energy plot (time domain)
plot(abs(x));

% preamble detection
for n = 1 : length(x)
    if n < length(x) - length(preamble_seqTime)
        cor_hist(n) = dot(preamble_seqTime,x(n:n+fftSize-1));
    end 
end

figure(1); %correlation plot
hold on;
plot(abs(cor_hist),'r');
hold off;
[dum max_timing] = max(abs(cor_hist));

% peak to average calculation
abs(cor_hist(max_timing).^2)/mean(abs(cor_hist(max_timing-500:max_timing+500-1)).^2)

% premable analysis : frequency domain
preambleSym_f = fft(x(max_timing : max_timing+fftSize-1),fftSize);
preambleSym_f = [preambleSym_f(fftSize/2+1:fftSize) preambleSym_f(1:fftSize/2)];

figure(2); %power spectral density
plot(abs(preambleSym_f).^2);

% SNR calculation from preamble
preamble_index = [fftSize/2-floor(length(preamble_seq_real)/2)+2:fftSize/2+floor(length(preamble_seq_real)/2)-1];
preamble_sigPow = mean(abs(preambleSym_f(preamble_index)).^2);
noise_index = [fftSize/2-floor(length(preamble_seq_real)/2)-100: fftSize/2-floor(length(preamble_seq_real)/2)];
preamble_noisePow = mean(abs(preambleSym_f(noise_index)).^2);
preamble_SNR = 10*log10((preamble_sigPow-preamble_noisePow)/preamble_noisePow)


% Maximum likelihood freq offset estimation-------------------
if option_MLfreqOffsetEst == 1
    min_MSE = 10000000;
    min_fOff = -1;
    for fOff = [-1500 : 10 : 1500];

        % Calculate symbol offset
        frame_start = max_timing - nbcp - 6*(nbcp+fftSize) - nbcp0;
        symb0_offset = frame_start+nbcp0+nbcp;
        dmrs0_offset = frame_start+nbcp0+nbcp + 3*(fftSize+nbcp);
     
        % offset compensation
        for i = 1 : nbcp0 + (nbcp + fftSize)*6
            x_comp(i) = x(i) * exp(1j*2*pi*fOff*i/samplingRate);
        end
        
       sym0_f = fft(x_comp(symb0_offset: symb0_offset+fftSize-1));
       dmrs0_f = fft(x_comp(dmrs0_offset: dmrs0_offset+fftSize-1));
        
        % pilot generation
        pilot0 = genPilot(Ns, 3, 0, Ndata);     
        
        % zero forcing estimation
        dmrs0_f_ext = dmrs0_f(fOffset+1:fOffset+Ndata);
        h0 = dmrs0_f_ext.*conj(pilot0);

        for i = 1 : Ndata
            subc_index = mod(fOffset+i,fftSize);
            decodedData(i) = conj(h0(i))*sym0_f(subc_index);
        end

        % ML detection
        for k = 1 : Ndata
            max_metric = -1;
            max_index = -1;
            for i = 1 : length(QPSKsym)
                metric = QPSKsym(i).* conj(decodedData(k));
                if max_metric < metric
                    max_metric = metric;
                    max_index = i;
                end
            end
            hddata(k) = QPSKsym(max_index);
        end

        % MSE calculation
        MSE = 0;
        for k = 1 : Ndata
            MSE = MSE + (angle(hddata(k)) - angle(decodedData(k))).^2;
        end

        if min_MSE > MSE
            min_MSE = MSE;
            min_fOff = fOff;
        end
    end

    min_MSE
    min_fOff

% offset compensation
    for i = 1 : nbcp0 + (nbcp + fftSize)*6
        x(i) = x(i) * exp(1j*2*pi*min_fOff*i/samplingRate);
    end

end

% Calculate symbol offset
frame_start = max_timing - 6*(nbcp+fftSize) - (nbcp + nbcp0);
symb0_offset = frame_start+nbcp0+nbcp;
dmrs0_offset = frame_start+nbcp0+nbcp + 3*(fftSize+nbcp);

% TDP -> FDP conversion
sym0_f = fft(x(symb0_offset: symb0_offset+fftSize-1));
dmrs0_f = fft(x(dmrs0_offset: dmrs0_offset+fftSize-1));

% pilot generation
pilot0 = genPilot(Ns, 3, 0, Ndata);

% zero forcing estimation
dmrs0_f_ext = dmrs0_f(fOffset+1:fOffset+Ndata);
h0 = dmrs0_f_ext.*conj(pilot0);

figure(3);
plot(angle(h0));

for i = 1 : Ndata
    subc_index = mod(fOffset+i,fftSize);
    decodedData(i) = conj(h0(i))*sym0_f(subc_index);
    
    if option_offsetComp == 1
        decodedData(i) = decodedData(i)*exp(-j*pi/2);
    end
end

% ML detection
for k = 1 : Ndata
    max_metric = -1;
    max_index = -1;
    for i = 1 : length(QPSKsym)
        metric = QPSKsym(i).* conj(decodedData(k));
        if max_metric < metric
            max_metric = metric;
            max_index = i;
        end
    end
    hddata(k) = QPSKsym(max_index);
end

figure(4);
scatter(real(decodedData), imag(decodedData));

%SNR estimation
sigPower = mean(abs(decodedData).^2);

for i = 1 : Ndata
    nVec(i) = hddata(i)*sqrt(sigPower) - decodedData(i);
end

snr = 10*log10(sigPower / mean(abs(nVec).^2))

% frequency offset estimation : 2nd step
h_est = angle(h0)/(fftSize+nbcp)*samplingRate/2/pi

HDDATA = convertFixPoint(hddata);
DECODEDATA = convertFixPoint(decodedData);

datanum = [0,0,0,0,0,0,0,0,0,0];
dec = [0,0,0,0,0,0,0,0,0,0];

for i = 1 : 10
    
    if(DECODEDATA(i) > 0)
        datanum(i) = 1;
    else
        datanum(i) = 0;
    end
    
    dec(i) = datanum(i)*2^(10-i);
end

sum = dec(1)+ dec(2)+ dec(3)+ dec(4)+dec(5)+dec(6)+dec(7)+dec(8)+dec(9)+dec(10)