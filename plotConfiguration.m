function fig = plotConfiguration(thrust, wings, fuselages, tails)
% Function plotConfiguration displays a simplified representation of the
% positions and mounting angles of each structural component defining a
% configuration. fig is the output figure.
%
% thrust: vector containing all 'thrust' objects present in the configuration.
%         configuration. These may be any thrust vectoring component or
%         system e.g. rotors or ducts, defined as subclasses.
%
% wings: vector containing all wing objects in the configuration. Includes
% only primary lifting surfaces, with integrated control surfaces.
%
% 
%
% Jemima Poynton 06/02/24
