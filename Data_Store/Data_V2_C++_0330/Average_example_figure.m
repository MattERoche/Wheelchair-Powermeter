clear all; close all all;
t = linspace(-pi,pi,100);
rng default  %initialize random number generator
x = sin(t) + 0.1*rand(size(t));
z = sin(t);
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y = filter(b,a,x);
zy = filter(b,a,z);

figure
plot(t,x)
hold on
grid on
plot(t,y)
plot(t,z)
%plot(t,zy)
legend('Input Data','Filtered Data','Exact Waveform')
hold off
figure
plot(t,z)
hold on
grid on
plot(t,zy)
legend('Exact Waveform','Filtered Waveform')