%//#############################################################################
%//! \file /2837x_FIR/matlab/FIR32forC28x.m
%//!
%//! \brief  MATLAB code for the Finite Impulse Response Filter
%//! \author C2000
%//! 
%//  Group:             C2000
%//  Target Family:     x86
%//
%// Copyright (C) $YEAR$ Texas Instruments Incorporated - http://www.ti.com/ 
%// ALL RIGHTS RESERVED 
%//#############################################################################
%//$TI Release: $
%//$Release Date: $
%//#############################################################################

clc
clear all
close all

FREQ1 = 1e3;    %1KHz signal
FREQ2 = 33e3;   %33KHz signal
SAMPLING_FREQ = 100e3; %100KHz

FIR_ORDER = 63;
BUFFER_SIZE = (FIR_ORDER+1) * 4;

k1 = FREQ1/SAMPLING_FREQ;
k2 = FREQ2/SAMPLING_FREQ;

i = 1 : 1 : BUFFER_SIZE;

S = 0.5*(sin(2*pi*k1*i) + sin(2*pi*k2*i));

N     = FIR_ORDER;   % Order
Fpass = 0.07;  % Passband Frequency
Fstop = 0.15;  % Stopband Frequency
Wpass = 1;    % Passband Weight
Wstop = 1;    % Stopband Weight
dens  = 20;   % Density Factor

% Calculate the coefficients using the FIRPM function.
% b is the parameter for the filter
b  = firpm(N, [0 Fpass Fstop 1], [1 1 0 0], [Wpass Wstop], {dens});
Hd = dfilt.dffir(b);

Fc = Hd.Numerator;

S_filt = conv(S,Fc);
S_filt = S_filt(1:BUFFER_SIZE);

%plot of the signal
subplot(1,2,1),plot(S),grid on;
axis([0 BUFFER_SIZE-1 -2 2]);
title('Signal')
xlabel('time (microseconds)')

%plot of the filtered signal
subplot(1,2,2),
plot(S_filt),grid on;
axis([0 BUFFER_SIZE-1 -2 2]);
title('LPF Signal')
xlabel('time (microseconds)')


% End of file
