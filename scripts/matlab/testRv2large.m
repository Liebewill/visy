function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(9);
w=2;
   dataset = 'IROS-TW';

  ft1='test_0.05_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;10;101;(0.1,0.15,0.2,0.25,0.3);DF_B3DV2.csv';
t1=readResults(dataset,ft1);
plotResults(t1,colors(1,:),w);hold on;
ft2='test_0.05_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;11;101;(0.1,0.15,0.2,0.25,0.3);DF_B3DV2.csv';
t2=readResults(dataset,ft2);
plotResults(t2,colors(2,:),w);hold on;
ft3='test_0.05_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.1,0.15,0.2,0.25,0.3);DF_B3DV2.csv';
t3=readResults(dataset,ft3);
plotResults(t3,colors(3,:),w);hold on;
ft4='test_0.05_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;7;101;(0.1,0.15,0.2,0.25,0.3);DF_B3DV2.csv';
t4=readResults(dataset,ft4);
plotResults(t4,colors(4,:),w);hold on;
ft5='test_0.05_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;8;101;(0.1,0.15,0.2,0.25,0.3);DF_B3DV2.csv';
t5=readResults(dataset,ft5);
plotResults(t5,colors(5,:),w);hold on;
ft6='test_0.05_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;9;101;(0.1,0.15,0.2,0.25,0.3);DF_B3DV2.csv';
t6=readResults(dataset,ft6);
plotResults(t6,colors(6,:),w);hold on;

   xlabel('1-precision');
    ylabel('recall');
   legend('10','11','12','4','5','6','7','8','9', 'Location','northwest');

end

