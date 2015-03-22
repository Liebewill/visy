function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(9);
w=2;
   dataset = 'IROS-TW';
   
   plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(1,:),w,'-.');hold on;
   plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(2,:),w,'-.');hold on;
   plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(3,:),w,'-.');hold on;
   
   
   plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(4,:),w);hold on;
   plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(5,:),w);hold on;
   plotResults(readResults(dataset,'test_0.05_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(6,:),w);hold on;


   xlabel('1-precision');
    ylabel('recall');
   legend('MV2 12 3D','MV2 10 3D','MV2 8 3D','MV2 12 1D','MV2 10 1D','MV2 8 1D', 'Location','northwest');
    grid on;
end

