% DTMF Decoder in MATLAB
clear all; close all; clc;

% Sampling parameters
Fs = 8000;          % Sampling frequency (Hz)
duration = 0.5;     % Duration of each tone (s)
t = 0:1/Fs:duration-1/Fs; % Time vector

% DTMF frequencies (Hz)
row_freqs = [697, 770, 852, 941];
col_freqs = [1209, 1336, 1477, 1633];

% Keypad mapping
keys = ['1','2','3','A';
        '4','5','6','B';
        '7','8','9','C';
        '*','0','#','D'];

% Generate a test DTMF signal (e.g., '5' = 770Hz + 1336Hz)
row_freq = 770;     % Change to test different keys
col_freq = 1336;
dtmf_signal = sin(2*pi*row_freq*t) + sin(2*pi*col_freq*t);

% Add noise (optional)
noise = 0.1*randn(size(t));
dtmf_signal = dtmf_signal + noise;

% Play the tone (optional)
sound(dtmf_signal, Fs);
pause(duration + 0.5);

% Goertzel Algorithm for frequency detection
function mag = goertzel(signal, target_freq, Fs)
    N = length(signal);
    k = round(target_freq * N / Fs);
    w = 2 * pi * k / N;
    cosine = cos(w);
    sine = sin(w);
    coeff = 2 * cosine;
    
    q1 = 0;
    q2 = 0;
    
    for n = 1:N
        q0 = coeff * q1 - q2 + signal(n);
        q2 = q1;
        q1 = q0;
    end
    
    real_part = q1 - q2 * cosine;
    imag_part = q2 * sine;
    mag = sqrt(real_part^2 + imag_part^2);
end

% Detect row and column frequencies
row_mags = zeros(1,4);
col_mags = zeros(1,4);

for i = 1:4
    row_mags(i) = goertzel(dtmf_signal, row_freqs(i), Fs);
    col_mags(i) = goertzel(dtmf_signal, col_freqs(i), Fs);
end

% Find the strongest frequencies
[~, row_idx] = max(row_mags);
[~, col_idx] = max(col_mags);

% Decode the key
decoded_key = keys(row_idx, col_idx);
disp(['Detected Key: ', decoded_key]);

% Plot the results
figure;
subplot(2,1,1);
plot(t, dtmf_signal);
title('DTMF Signal (Time Domain)');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(2,1,2);
stem([row_freqs, col_freqs], [row_mags, col_mags], 'filled');
title('Goertzel Frequency Detection');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
grid on;