% Simulate data decoding 
clc; clear all; close all;

fftSize = 1024;
nbcp = 72;
nbcp0 = 16;

% load data 
x = convertToReal(load('./rxdata_decoding.txt'));

% pilot gentration 
gendmrs = genPilot(2, 3, 0, 12);
gendmrs_short =convertFixPoint(gendmrs);

% preamble signal generation (freq domain signal generation)
preamble_seq = [4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0];
preamble_seq_real = convertToReal(preamble_seq);

lowerCnt = floor(length(preamble_seq_real)/2)+1;
upperCnt = length(preamble_seq_real)-lowerCnt;

preamble_seq_freq = zeros(1,fftSize);
preamble_seq_freq(fftSize- lowerCnt+1: fftSize) = preamble_seq_real(1:lowerCnt); 
preamble_seq_freq(1:upperCnt) = preamble_seq_real(length(preamble_seq_real) - upperCnt+1:length(preamble_seq_real));

% time domain signal
preamble_seqTime = ifft(preamble_seq_freq, fftSize);

% preamble detection
for n = 1 : length(x)
    if n < length(x) - length(preamble_seqTime)
        o_cor_hist(n) = dot(preamble_seqTime,x(n:n+fftSize-1));
    end
end

% correlation plot
figure(1); 
plot(abs(x));
hold on;
plot(abs(o_cor_hist),'r');
hold off;

% return max timing  
[dum o_max_timing] = max(abs(o_cor_hist));

% peak to average calculation
abs(o_cor_hist(o_max_timing).^2)/mean(abs(o_cor_hist(o_max_timing-500:o_max_timing+500-1)).^2)

% preamble analysis : frequency domain
preambleSym_f = fft(x(o_max_timing : o_max_timing +fftSize-1),fftSize);
preambleSym_f = [preambleSym_f(fftSize/2+1:fftSize) preambleSym_f(1:fftSize/2)]; 

% Convert pilot from frequency axis to time axis
dmrs_start = o_max_timing -3*(fftSize+nbcp); 

dmrsSym_f = fft(x(dmrs_start:dmrs_start+fftSize-1),fftSize);
dmrsSym_f = [dmrsSym_f(fftSize/2+1:fftSize) dmrsSym_f(1:fftSize/2)];
dmrsSym_f_short = convertFixPoint(dmrsSym_f);

% Convert data from frequency axis to time axis
dataSym_start = o_max_timing -6*(fftSize+nbcp);

dataSym_f = fft(x(dataSym_start:dataSym_start+fftSize-1),fftSize);
dataSym_f = [dataSym_f(fftSize/2+1:fftSize) dataSym_f(1:fftSize/2)];

% SNR calculation from preamble
preamble_index = [fftSize/2-floor(length(preamble_seq_real)/2)+2:fftSize/2+floor(length(preamble_seq_real)/2)-1];
preamble_sigPow = mean(abs(preambleSym_f(preamble_index)).^2);
noise_index = [fftSize/2-floor(length(preamble_seq_real)/2)-100: fftSize/2-floor(length(preamble_seq_real)/2)];
preamble_noisePow = mean(abs(preambleSym_f(noise_index)).^2);
preamble_SNR = 10*log10((preamble_sigPow-preamble_noisePow)/preamble_noisePow)

% Channel Estimation
pilot_H = dmrsSym_f(213:224).* conj(gendmrs(1:12));
short_pilot_H = convertFixPoint(pilot_H);

% compensation
comp_dataSym_f = dataSym_f(213:217).*conj(pilot_H(1:5));
dec_data = convertFixPoint(comp_dataSym_f./abs(comp_dataSym_f))
comp_dataSym_f_short =convertFixPoint(dataSym_f);
