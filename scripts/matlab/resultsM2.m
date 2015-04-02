function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(5);
w=3;
   dataset = 'IROS-TW';

  

plotResults(readResults(dataset,'test_0.01_DETECTOR_BOLD!!.csv'),colors(1,:),w);hold on;
plotResults(readResults(dataset,'test_0.01_DETECTOR_B3D_R_3ANGLE!EXTRACTOR_BOLD3D;E101;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.1);DF_B3DV2.csv'),colors(2,:),w);hold on;
plotResults(readResults(dataset,'test_0.01_DETECTOR_B3D_R_2ANGLE!EXTRACTOR_BOLD3D;E101;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.1);DF_B2D.csv'),colors(3,:),w);hold on;
plotResults(readResults(dataset,'test_0.01_DETECTOR_B3D_R_4ANGLE!EXTRACTOR_BOLD3D;E101;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.1);DF_B3D4H.csv'),colors(4,:),w);hold on;
plotResults(readResults(dataset,'test_0.01_DETECTOR_B3D_R_FPFH!EXTRACTOR_BOLD3D;E101;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.1);DF_FPFH.csv'),colors(5,:),w);hold on;



%plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(7,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('Bold','R 3Angle','R 2Angle','R 4Angle','R FPFH', 'Location','northwest');

end

