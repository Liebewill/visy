function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(12);
w=3;
   dataset = 'IROS-TW';

  
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(1,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(2,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(3,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;6;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(4,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(5,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(6,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(7,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2MULTI!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;6;100;(5,10,15,20,25);DF_B3DV2-MULTI.csv'),colors(8,:),w);hold on;


plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(9,:),w,'--');hold on;

%plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(7,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('M2_H3D 12','M2_H3D 10','M2_H3D 8','M2_H3D 6','M2_H3D 12 Occ','M2_H3D 10 Occ','M2_H3D 8 Occ','M2_H3D 6 Occ','Bold', 'Location','northwest');

end

