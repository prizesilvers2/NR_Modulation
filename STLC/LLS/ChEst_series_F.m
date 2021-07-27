% Calculate preamble Channels Continuously
clc; clear all; close all;

Size = 200;
fftSize = 1024;

% wireless -> 0 cable -> 1
connection_option = 1;

figure_option = 0;


if connection_option == 0
    filepath_rx = './rx_preamble000.txt';
    filepath_tx = './tx_preamble000.txt';
else
    filepath_rx = './rx_preamble_cable000.txt';
    filepath_tx = './tx_preamble_cable000.txt';
end

%preamble sequence pattern 
preamble_seq = [4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0];
preamble_seq_real = convertToReal(preamble_seq);

for file_num = 1 : 2
    file_num
    
    if connection_option == 0
        filepath_rx(14) = '0'+floor(file_num/100);
        filepath_tx(14) = '0'+floor(file_num/100);
        filepath_rx(15) = '0'+mod(floor(file_num/10),10);
        filepath_tx(15) = '0'+mod(floor(file_num/10),10);
        filepath_rx(16) = '0'+mod(file_num,10);
        filepath_tx(16) = '0'+mod(file_num,10);
    else
        filepath_rx(20) = '0'+floor(file_num/100);
        filepath_tx(20) = '0'+floor(file_num/100);
        filepath_rx(21) = '0'+mod(floor(file_num/10),10);
        filepath_tx(21) = '0'+mod(floor(file_num/10),10);
        filepath_rx(22) = '0'+mod(file_num,10);
        filepath_tx(22) = '0'+mod(file_num,10);    
    end
    
    x = convertToReal(load(filepath_rx)); %DL (dump rx)
    y = convertToReal(load(filepath_tx)); %UL (dump tx)

    for i_data = 1 : Size
        x_preambleSym_f = x((i_data-1).*1024 +1 : i_data.*1024);
        y_preambleSym_f = y((i_data-1).*1024 +1 : i_data.*1024);

        x_preambleSym_f = [x_preambleSym_f(fftSize/2+1:fftSize) x_preambleSym_f(1:fftSize/2)]; 
        y_preambleSym_f = [y_preambleSym_f(fftSize/2+1:fftSize) y_preambleSym_f(1:fftSize/2)]; 

        %calculate preamble channel H 
        x_preamble_H = x_preambleSym_f(449:575).* conj(preamble_seq_real(1:127));
        y_preamble_H = y_preambleSym_f(449:575).* conj(preamble_seq_real(1:127));

        diff = mean(unwrap(angle(x_preamble_H)))-mean(unwrap(angle(y_preamble_H)));
        
        while (diff > 2*pi) || (diff < -2*pi) 
            if diff > 2*pi
                diff = diff - 2*pi;
            elseif diff < -2*pi
                diff = diff + 2*pi;
            end
        end

        hist_diff(Size*(file_num-1)+i_data) = diff;

        if figure_option == 1
            figure(Size*(file_num-1)+i_data);
            plot(unwrap(angle(y_preamble_H)),'r'); %UL
            hold on;
            plot(unwrap(angle(x_preamble_H)),'b'); %DL
            hold off;
        end
    end
end
figure(100000);
set(100000,'color','w')
histogram(hist_diff,10);
