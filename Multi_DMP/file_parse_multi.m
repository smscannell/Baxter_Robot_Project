function [ tau,w1,e1 ] = file_parse_multi( col_t, col_w1, col_e1 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Read csv file
data = csvread('pool_shot',1);
w1 = data(:,col_w1);  % w1 joint position
e1 = data(:,col_e1);  % e1 joint position
t = data(:,col_t);  % t = timestamp

% Trim data
plot(w1); hold on; plot(e1); hold off;
lower = input('Please input lower bound ');
higher = input('Please input higher bound ');
w1 = w1(lower:higher); % Trim data recording
e1 = e1(lower:higher);
t = t(lower:higher);
tau = round(t(end)-t(1),2);
close all

end

