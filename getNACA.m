function [NACA_Data, Cla, tc, shape] = getNACA(name, plotOut)
% function getNACA obtains and formats airfoil data.
% 
% name: string or character array containing the airfoil type in some
%       variation of the form NACA xxxx for a 4 - 6 digit series aerofoil.
%       The first series of digits is taken as the aerofoil code.
%
% plotOut: boolean where plotOut == 1 plots the airfoil data, and plotOut 
%          == 0 does not
%
% README: this data has been sourced directly from
% https://splinecloud.com/repository/AlinaNeh/NACA_airfoil_database/, which
% is a repository containing formatted data from http://airfoiltools.com/.
% The data has been verified against
% https://ntrs.nasa.gov/api/citations/19930090976/downloads/19930090976.pdf
% where available.

%% Extract key details from 'name'
% Handles various inputs e.g. 0012, NACA0012, NACA 0012 by assuming that the 
% series of digits is the airfoil code, regardless if they are continuous
% to account for seperation (e.g. dashes)

if isnumeric(name)
   error('Please input the NACA code as a string.') 
end

name = char(name); % ensures that name is in indexed char form

digits = regexp(name, '\d+', 'match');
code = char(digits(1)); % convert from (1,2) cell to char array of first element

N = length(code); % Number of digits
tc = str2double("0."+ code(end-1:end));

%% Get data
filepath = fileparts(mfilename('fullpath')); % Get current filepath (assume dataset with function)

try
    % Find dataset
    if N == 4
       load([filepath '\NACA Data\4-Digit Series'], 'S')
       load([filepath '\NACA Data\4-Digit Series Shapes'], 'shape')
    elseif N == 5
        load([filepath '\NACA Data\5-Digit Series'], 'S')
        load([filepath '\NACA Data\5-Digit Series Shapes'], 'shape')
    elseif N == 6
        load([filepath '\NACA Data\6-Digit Series'], 'S')
        load([filepath '\NACA Data\6-Digit Series Shapes'], 'shape')
    else
        error('Airfoil not found in database. Please manually assign experimental data to property obj.NACA_Data.')
    end

S_filenames = {S.name}; % gets filenames in cell format
shape_filenames = {shape.name};

idx = contains(S_filenames, code) == 1; % find datasets associated with code
idx_shape = find(contains(shape_filenames, code) == 1);
shape = shape(idx_shape).data;

dataset = S(idx);
NACA_Data = struct(); % create data structure for readability

for i = 1:length(dataset) % Reformat the data
    data = dataset(i).data;

    digits = regexp(dataset(i).name, '\d+', 'match'); % find all digits

    Re = str2double(digits{:,end}); % get Reynolds number from filename, convert to double

    NACA_Data(i).alpha = data(:,1);
    NACA_Data(i).Cl = data(:,2);
    NACA_Data(i).Cd = data(:,3);
    NACA_Data(i).Cm = data(:,5);
    NACA_Data(i).Re = Re;
end

if plotOut == 1 % plot the data
    fig1 = figure(); hold on; grid on;
    fig2 = figure(); hold on; grid on;
    fig3 = figure(); hold on; grid on;
    fig4 = figure(); hold on; grid on;
    fig5 = figure(); hold on; grid on;

    Re_nums = [];
end

for i = 1:length(NACA_Data)
    if plotOut == 1
        figure(fig1);
        plot(NACA_Data(i).alpha, NACA_Data(i).Cl);
        Re_nums = [Re_nums ("Re " + string(NACA_Data(i).Re))];
        legend(Re_nums, 'location', "northwest")
        xlabel('Angle of Attack (deg)'); ylabel('Lift Coefficient'); 

        figure(fig2);
        plot(NACA_Data(i).alpha, NACA_Data(i).Cd);
        legend(Re_nums, 'location', "northwest")
        xlabel('Angle of Attack (deg)'); ylabel('Drag Coefficient'); 

        figure(fig3);
        plot(NACA_Data(i).alpha, NACA_Data(i).Cl./NACA_Data(i).Cd);
        legend(Re_nums, 'location', "northwest")
        xlabel('Angle of Attack (deg)'); ylabel('Lift to Drag Ratio (Cl/Cd)'); 

        figure(fig4);
        plot(NACA_Data(i).alpha, NACA_Data(i).Cm);
        legend(Re_nums, 'location', "northwest")
        xlabel('Angle of Attack (deg)'); ylabel('Pitching Moment Coefficient'); 

        figure(fig5)
        plot(shape(:,1), shape(:,2))
        xlabel('Chord ratio'); ylabel('Thickness')
    end

    midpoint = round(length(NACA_Data(i).alpha)/2,0); % midpoint of data

    gradient = diff(NACA_Data(i).Cl)./diff(NACA_Data(i).alpha*(pi/180));
    changeidx1 = find(ischange(gradient(1:midpoint)) == 1);
    changeidx2 = find(ischange(gradient(midpoint:end)) == 1);

    [~, idx1] = max(diff(changeidx1));
    [~, idx2] = max(diff(changeidx2));

    Cla(i) = mean([gradient(changeidx1(idx1):midpoint); gradient(changeidx2(idx2+1):midpoint)]); % include middle region to account for low Re behaviour
end 

Cla = mean(Cla); % Find mean from all Re

catch % handle error case
    error('Airfoil not found in database. Please manually assign experimental data to property obj.NACA_Data.')    
end