function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(3);
w=3;
   dataset = 'IROS-TW';

  

%plotResults(readResults(dataset,'test_0.03_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(1,:),w);hold on;
plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(2,:),w,'-.');hold on;

%plotResults(readResults(dataset,'test_0.03_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(2,:),w);hold on;

plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD!!.csv'),colors(3,:),w,'--');hold on;

%plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(7,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('M2 12 Occlusion','Bold', 'Location','northwest');
    grid on;
end

