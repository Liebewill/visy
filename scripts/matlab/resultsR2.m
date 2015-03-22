function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(12);
w=3;
   dataset = 'IROS-TW';

  

plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;14;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2.csv'),colors(5,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;16;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2.csv'),colors(6,:),w);hold on;

plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;14;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2.csv'),colors(1,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;16;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2.csv'),colors(2,:),w);hold on;

plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(11,:),w,'--');hold on;

%plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(7,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('R2 14','R2 16','R2 14 Occ','R2 16 Occ','Bold', 'Location','northwest');

end

