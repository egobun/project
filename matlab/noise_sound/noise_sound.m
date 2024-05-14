[y,Fs] = audioread("Dance_Monkey.mp3");
[x,Fs] = audioread("Dance_Monkey.mp3",[1,length(y) - 1*Fs]);
N = length(x);

sound(x,Fs);
pause(1);
clear sound;
xn = awgn (x, 15, 'measured');

xden = wdenoise(xn, 'DenoisingMethod', 'Bayes', 'ThresholdRule', 'Soft', 'NoiseEstimate', 'LevelIndependent', 8, 'Wavelet', 'sym8');

subplot (3,1,1)

plot(x);

title ('Исходный аудиосигнал');

subplot (3,1,2)

plot (xn, 'r');

title ('Зашумленный аудиосигнал');

subplot (3,1,3)

plot (xden, 'b');

title ('Очищенный аудиосигнал');

figure (2)

plot (xn, 'r')

hold on

plot (xden, 'b')
sound(xn,Fs);
pause(1);
clear sound;
grid on

legend ("Зашумленный аудиосигнал", "Очищенный аудиосигнал")

hold off

figure
cwt(x,'amor',100)