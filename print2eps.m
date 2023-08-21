function [] = print2eps(fig,file_name,width,height,renderer)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% 'renderer' options:
%   'opengl' — OpenGL® renderer. This option enables MATLAB to access
%   graphics hardware if it is available on your system. The OpenGL
%   renderer displays objects sorted in front to back order, as seen on the
%   monitor. Lines always draw in front of faces when at the same location
%   on the plane of the monitor.
%
%   'painters' — Painters renderer. This option works well for axes in a
%   2-D view. In 2-D, the Painters renderer sorts graphics objects by child
%   order (order specified). In 3-D, the Painters renderer sorts objects in
%   front to back order. However, it might not correctly draw intersecting
%   polygons in 3-D.

set(fig, 'PaperPositionMode','manual');
set(fig,'paperunits','centimeters')
set(fig,'papersize',[width,height])
set(fig,'paperposition',[0,0,width,height])
set(fig,'renderer',renderer)
print(file_name,'-depsc')

end