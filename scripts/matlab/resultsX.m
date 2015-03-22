function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(12);
w=3;
   dataset = 'IROS-TW';

plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DRX!EXTRACTOR_SIFT!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2-MULTI.csv'),colors(1,:),w,'--');hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DRX!EXTRACTOR_SIFT!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2-MULTI.csv'),colors(2,:),w,'--');hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DRX!EXTRACTOR_SIFT!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2-MULTI.csv'),colors(3,:),w,'--');hold on;
plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD3DRX!EXTRACTOR_SIFT!DESCRIPTOR_BOLD3D-MULTIBUNCH;6;101;(0.1,0.2,0.3,0.4,0.5);DF_B3DV2-MULTI.csv'),colors(4,:),w,'--');hold on;


plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(11,:),w,'--');hold on;

%plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(7,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('X 12','X 10','X 8','X 6', 'Bold', 'Location','northwest');

end

