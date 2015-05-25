function [ output_args ] = resultsHVZ( input_args )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
    
colors=hsv(4);
w=3;
   dataset = 'TW';

%plotResultsHV(readResults(dataset,'test_HOUGHCOMPARE_0.01_SIFT.csv'),colors(1,:),w,'--');hold on;
plotResultsHV(readResults(dataset,'test_HOUGHCOMPARE_05_0.05_SIFT.csv'),colors(1,:),w,'--');hold on;

%plotResultsHV(readResults(dataset,'test_HOUGHCOMPARE_0.01_BOLD.csv'),colors(2,:),w,'--');hold on;
plotResultsHV(readResults(dataset,'test_HOUGHCOMPARE_05_0.05_BOLD.csv'),colors(2,:),w,'--');hold on;


%plotResultsHV(readResults(dataset,'test_HOUGHCOMPARE_NOHOUGH_0.01_B3D_R_Z.csv'),colors(3,:),w,'--');hold on;
plotResultsHV(readResults(dataset,'test_HOUGHCOMPARE_05__NOHOUGH_0.05_B3D_R_Z.csv'),colors(3,:),w,'--');hold on;

plotResultsHV(readResults(dataset,'test_HOUGHCOMPARE_HOUGH_0.05_B3D_R_Z.csv'),colors(4,:),w,'--');hold on;


   xlabel('1-precision');
    ylabel('recall');
   legend('SIFT','BOLD','B3D Z GC','B3D Z HOUGH3D', 'Location','northwest');

end

