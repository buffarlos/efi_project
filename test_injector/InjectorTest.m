clc;
close all;
clear;

injectorData = readtable('InjectorTest.csv');


volumePerInjection_40psi = [];
times_40psi = [];
volumePerInjection_50psi = [];
times_50psi = [];

for n = 1:size(injectorData)
    if injectorData.Pressure(n) == 40
        volumePerInjection_40psi = [volumePerInjection_40psi injectorData.Volume(n) / injectorData.Events(n)];
        times_40psi = [times_40psi injectorData.InjectionTime(n)];
    elseif injectorData.Pressure(n) == 50
        volumePerInjection_50psi = [volumePerInjection_50psi injectorData.Volume(n) / injectorData.Events(n)];
        times_50psi = [times_50psi injectorData.InjectionTime(n)];    
    end
end

figure;
scatter(times_40psi, volumePerInjection_40psi);
title('Injection Volume vs Injection Time, 40psi');

figure;
scatter(times_50psi, volumePerInjection_50psi);
title('Injection Volume vs Injection Time, 50psi');

