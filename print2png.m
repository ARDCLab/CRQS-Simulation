function [] = print2png(fig,file_name,width,height)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

set(fig, 'PaperPositionMode','manual');
set(fig,'paperunits','centimeters')
set(fig,'papersize',[width,height])
set(fig,'paperposition',[0,0,width,height])
set(fig,'renderer','opengl')
print(file_name,'-dpng','-r300')

end