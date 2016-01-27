function [ t,x ] = file_parse( col_t, col_x )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Read csv file
data = csvread('pool_shot',1);
x = data(:,col_x);  % w1 joint position
t = data(:,col_t);  % t = timestamp

% Trim data
plot(x);
lower = input('Please input lower bound ');
higher = input('Please input higher bound ');
x = x(lower:higher); % Trim data recording
close all

end

