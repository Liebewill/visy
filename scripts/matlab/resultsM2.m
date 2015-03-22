function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(12);
w=3;
   dataset = 'IROS-TW';

  

plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;6;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(5,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(6,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(1,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(2,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;14;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(3,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;16;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(4,:),w);hold on;

plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;6;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(11,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(12,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(7,:),w);hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(8,:),w,'-.');hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;14;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(9,:),w,'-.');hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;0;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;16;100;(5,10,15,20,25);DF_B3DV2.csv'),colors(10,:),w);hold on;


plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(11,:),w,'--');hold on;

%plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(7,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('M2 6','M2 8','M2 10','M2 12','M2 14','M2 16','M2 6 OCC','M2 8 OCC','M2 10 OCC','M2 12 OCC','M2 14 OCC','M2 16 OCC','Bold', 'Location','northwest');

end

