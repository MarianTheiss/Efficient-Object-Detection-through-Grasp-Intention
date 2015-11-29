%function returns a lowpass and a highpass filtered signal.
%input: realtime samples, variable number of datasets possible. 1 sample
%per dataset, timeConstant = 1/cutoff frequency, lpOld=input of the last
%lowpass filtered signal
%output: lowpass, highpass
function [lp, hp] = lowAndHighpass(timeConstant, timeBetweenSamples, updateSamples, lpOld) 
    alpha=timeConstant/(timeConstant+timeBetweenSamples);
    lp=alpha*lpOld+ (1-alpha) * updateSamples;
    hp=updateSamples-lp;