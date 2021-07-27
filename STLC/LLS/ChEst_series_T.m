% Calculate preamble Channels Continuously

clc; clear all; close all;

Size = 100;
ylenth = 6590;
fftSize = 1024;

% RX -> f-domain, TX -> t-domain
filepath_rx = './rx_preamble_T000.txt';               
filepath_tx = './tx_preamble_T000.txt';

preamble_seq = [4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0];
preamble_seq_real = convertToReal(preamble_seq);

lowerCnt = floor(length(preamble_seq_real)/2)+1;
upperCnt = length(preamble_seq_real)-lowerCnt;

%preamble signal generation (freq domain signal generation)
preamble_seq_freq = zeros(1,fftSize);
preamble_seq_freq(fftSize- lowerCnt+1: fftSize) = preamble_seq_real(1:lowerCnt); 
preamble_seq_freq(1:upperCnt) = preamble_seq_real(length(preamble_seq_real) - upperCnt+1:length(preamble_seq_real));

%time domain signal
preamble_seqTime = ifft(preamble_seq_freq, fftSize);

for file_num = 1 : 1
    file_num
    
    filepath_rx(16) = '0'+floor(file_num/100);
    filepath_tx(16) = '0'+floor(file_num/100);
    filepath_rx(17) = '0'+mod(floor(file_num/10),10);
    filepath_tx(17) = '0'+mod(floor(file_num/10),10);
    filepath_rx(18) = '0'+mod(file_num,10);
    filepath_tx(18) = '0'+mod(file_num,10); 
    
    x = convertToReal(load(filepath_rx));   %DL (dump rx)
    y = convertToReal(load(filepath_tx));   %UL (dump tx)

    for i_data = 1 : Size
        rx_preambleSym_f = x((i_data-1).*1024 +1 : i_data.*1024);        
        tx_preambleSym_t = y(1+(ylenth*(i_data-1)) : ylenth*i_data);
        
        for n = 1 : ylenth
            if n < ylenth - length(preamble_seqTime)
                cor_hist(n) = dot(preamble_seqTime,tx_preambleSym_t(n:n+fftSize-1));
            end
        end

        [dum max_timing] = max(abs(cor_hist));
        
        max_timing_buf(i_data) = max_timing;
        
        tx_preambleSym_f = fft(tx_preambleSym_t(max_timing : max_timing+fftSize-1),fftSize);
        tx_preambleSym_f = [tx_preambleSym_f(fftSize/2+1:fftSize) tx_preambleSym_f(1:fftSize/2)];
       
        rx_preambleSym_f = [rx_preambleSym_f(fftSize/2+1:fftSize) rx_preambleSym_f(1:fftSize/2)];         
       
        %calculate preamble channel H 
        rx_preamble_H = rx_preambleSym_f(449:575).* conj(preamble_seq_real(1:127));
        tx_preamble_H = tx_preambleSym_f(449:575).* conj(preamble_seq_real(1:127));

        diff = mean(unwrap(angle(rx_preamble_H)))-mean(unwrap(angle(tx_preamble_H)));
        
        while (diff > 2*pi) || (diff < -2*pi) 
            if diff > 2*pi
                diff = diff - 2*pi;
            elseif diff < -2*pi
                diff = diff + 2*pi;
            end
        end

        hist_diff(Size*(file_num-1-100)+i_data) = diff;

        figure(Size*(file_num-1)+i_data);
        plot(unwrap(angle(tx_preamble_H)),'b'); %UL
        hold on;
        plot(unwrap(angle(rx_preamble_H)),'r'); %DL
        hold off;
    end
end

figure(1000000);
histogram(hist_diff,10);
