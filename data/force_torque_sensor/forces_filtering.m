function [F_r, T_r, ft_bias] = forces_filtering(F_s, T_s, thetas, t, counter, m, l, fq, order, varargin)
% Describe function here
% This function is designed to work for the ATI mini45 sensor used on the
% end effector of the kuka youBot robot. It allows bias and gravity
% compensation, transformation of sensor data to the robot base frame and
% filtering of the data at 'fq' Hz
% /!\ the torque gravity compensation is not implemented (negligeable ?)
%
%DESCRIPTION:
%
%As input data, one has
%
%  F: a 3xN whos columns are the forces at N target times.
%  T: a 3xN whos columns are the torques at N target times.
%  thetas: a 5xN whos columns are the joints angle at N target times.
%  t: a N vector containing time information data
%  counter: an integer giving the number of samples to use to unbias data
%  m: estimated mass of the sensor, default value is 0.1096 kg
%  l: estimated arm lever of the sensor, default value is 0.0103 m
%  fq: cut off frequency for low pass filter, default value is 25 Hz
%  order: order of the butterworth low pass filter, default value is 4
%
%The basic syntax
%
%     []=forces_filtering(F_s, T_s, thetas, t) 
%
%find the bias of the force torque data
%
%    detail calculus...       
%
%
%with parameter/value pair options,
%
%  'rotation' -  Boolean flag. If TRUE, the data will be rotated to the
%  gobal frame
%
%  'gravity' -  Boolean flag. If TRUE, the gravitational effect of the sensor
%               will be deleted from the data.
%               
%  'filtering' -  Boolean flag. If TRUE, the sensor data will be filtered using
%                a low pass filter.
%
%  'plot' -  Boolean flag. If TRUE, plot the data before & after all signal 
%            process stages.
%
%
%OUTPUTS:
%
%
% detail outputs
%
%           text text text text text text
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Vincent FORTINEAU
% Copyright, Laboratoire des signaux et systèmes (L2S), France
%%Input option processing and set up
 options.rotation = 1;
 options.gravity = 1;
 options.filtering = 0; 
 options.doPlot = 0;
    
for ii=1:2:length(varargin)
    param = varargin{ii};
    val = varargin{ii+1};
    if strcmpi(param, 'rotation')
        options.rotation = val;
    elseif strcmpi(param, 'gravity')
        options.gravity = val;
    elseif strcmpi(param, 'filtering')
        options.filtering = val;
    elseif strcmpi(param, 'plot')
        options.doPlot = val;
    else
       error(['Option ''' param ''' not recognized']);
    end
end

rotation = options.rotation;
gravity = options.gravity;
filtering = options.filtering;
doPlot = options.doPlot;

% Default param
if ~exist('counter','var')
     % third parameter does not exist, so default it to something
      counter = 1000;
end
if ~exist('m','var')
     % third parameter does not exist, so default it to something
      m = 0.1096;
end
if ~exist('l','var')
     % third parameter does not exist, so default it to something
      l = 0.0103;
end  
if ~exist('fq','var')
     % third parameter does not exist, so default it to something
      fq = 25;
end 
if ~exist('order','var')
     % third parameter does not exist, so default it to something
      order = 4;
end 

%% Force calibration

% Bias estimation
FT_bias = zeros(6, counter);
grav = zeros(3, counter);
for i=1:counter
    T = MGD_T0handle(thetas(1,i), thetas(2,i), thetas(3,i), thetas(4,i), thetas(5,i));
    grav(:,i) = inv(T(1:3,1:3))*[0;0;-m*9.81];
    grav_tq(:,i) = cross(grav(:,i), [0;0;-m])*l;
    FT_bias(1:3, i) = F_s(:,i) - grav(:,i);
    FT_bias(4:6, i) = T_s(:,i) - grav_tq(:,i);
end
ft_bias = nanmean(FT_bias, 2);   
F_r = zeros(size(F_s));
T_r = zeros(size(T_s));
if rotation
    % disp('rotation')
    % Transformation from sensor to global reference
    for i=1:length(F_s)
        T = MGD_T0handle(thetas(1,i), thetas(2,i), thetas(3,i), thetas(4,i), thetas(5,i));
        F_r(:,i) = T(1:3,1:3) * (F_s(:, i) - ft_bias(1:3));
        T_r(:,i) = T(1:3,1:3) * (T_s(:, i) - ft_bias(4:6));
    end
else
    F_r(:,i) = F_s(:,i) - ft_bias(1:3);
    T_r(:,i) = T_s(:,i) - ft_bias(4:6);   
end

if (gravity == 'TRUE') | (gravity == 1) %TODO: compensate gravity torque
    if rotation
        % gravity compensation on Z axis
        F_r(3,:) = F_r(3,:) + m*9.81;
    else
        F_r = F_s - grav;
    end
end    

if (filtering == 'TRUE') | (filtering == 1)
    % filtering
    % TODO use filtfilt
    freq = length(t)/t(end);
    filter_order = order;
    fir_filter = fir1(filter_order, (fq/(freq/2)), 'low');

    F_r = filtfilt(fir_filter, 1, F_r'); 
    T_r = filtfilt(fir_filter, 1, T_r');
    %F_r = filter(fir_filter, 1, F_r'); 
    %T_r = filter(fir_filter, 1, T_r'); 
    %F_r = circshift(F_r, length(F_r) - filter_order/2, 1);
    %T_r = circshift(T_r, length(T_r) - filter_order/2, 1);
    
    if isnan(F_r)
        warning('NaN detected in forces, the filtering will populate NaNs !')
    elseif isnan(T_r)
        warning('NaN detected in torques, the filtering will populate NaNs !')
    end
    
    F_r = F_r';
    T_r = T_r';
end

if (doPlot == 'TRUE') | (doPlot == 1)
    figure()
    subplot(2,2,1)
    plot(t, F_s)
    grid on
    legend('x', 'y', 'z')
    title('Forces in sensor coordinates')
    subplot(2,2,2)
    plot(t, F_r)
    grid on
    legend('x', 'y', 'z')
    title('Forces after signal processing')
    subplot(2,2,3)
    plot(t, T_s)
    grid on
    legend('x', 'y', 'z')
    title('Torques in sensor coordinates')
    subplot(2,2,4)
    plot(t, T_r)
    grid on
    legend('x', 'y', 'z')
    title('Torques after signal processing')
end    

end

