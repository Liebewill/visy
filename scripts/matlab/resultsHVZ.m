function [ output_args ] = resultsHVZ( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(3);
w=3;
   dataset = 'TW';

%plotResultsHV(readResults(dataset,'test_10_DETECTOR_B3D_R_Z!EXTRACTOR_BOLD3D;E101;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.11);DF_B3DZ.csv'),colors(1,:),w,'--');hold on;
plotResultsHV(readResults(dataset,'test_hv_21_DETECTOR_B3D_R_Z!EXTRACTOR_BOLD3D;E101;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.1);DF_B3DZ.csv'),colors(1,:),w,'--');hold on;
plotResultsHV(readResults(dataset,'test_hv_20_DETECTOR_BOLD!!.csv'),colors(2,:),w,'--');hold on;

%plotResultsHV(readResults(dataset,'test_hv_10_DETECTOR_BOLD!!.csv'),colors(3,:),w,'-.');hold on;


%plotResultsHV(readResults(dataset,'test_10_DETECTOR_BOLD!!.csv'),colors(2,:),w,'--');hold on;


%plotResults(readResults(dataset,'test_0.02_DETECTOR_BOLD!!.csv'),colors(7,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('BOLD3D Z','BOLD', 'Location','northwest');

end

