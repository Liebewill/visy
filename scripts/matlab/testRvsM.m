function [ output_args ] = test1( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(9);
w=2;
   dataset = 'IROS-TW';

   ft1='test_0.05_DETECTOR_BOLD3DM2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25);DF_B3DV2.csv';
t1=readResults(dataset,ft1);
plotResults(t1,colors(1,:),w);hold on;
ft2='test_0.05_DETECTOR_BOLD3DM!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;100;(5,10,15,20,25).csv';
t2=readResults(dataset,ft2);
plotResults(t2,colors(2,:),w);hold on;
ft3='test_0.05_DETECTOR_BOLD3DR2!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.02,0.05,0.1,0.15,0.2);DF_B3DV2.csv';
t3=readResults(dataset,ft3);
plotResults(t3,colors(3,:),w);hold on;
ft4='test_0.05_DETECTOR_BOLD3DR!EXTRACTOR_BOLD3D;1;5;2;25;0.001!DESCRIPTOR_BOLD3D-MULTIBUNCH;12;101;(0.02,0.05,0.1,0.15,0.2).csv';
t4=readResults(dataset,ft4);
plotResults(t4,colors(4,:),w);hold on;


   xlabel('1-precision');
    ylabel('recall');
   legend('M v2','M v1','R v2','R v1', 'Location','northwest');
grid on;
end

