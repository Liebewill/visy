function [ Results ] = readResults( datasetName, filename )
    
    path = strcat('datasets/',datasetName,'/precision/',filename);
    
    file = fopen(path);
    line = fgetl(file);
    
    Results = [];
    while 1
        dl = regexp( line, ';', 'split' );
        p = str2double(dl(1));
        n = str2double(dl(2));
        tp = str2double(dl(3));
        fp = str2double(dl(4));
        tn = str2double(dl(5));
        fn = str2double(dl(6));
        precision = tp/(tp + fp);
        recall = tp/(tp + fn);
        v = [p,n,tp,fp,tn,fn,precision,1-precision,recall];
        
        Results = [Results; v];
        line = fgetl(file);
        if ~ischar(line), break, end
    end
    
    
    
%      file = fopen(file_path);
%     line = fgetl(file);
%     
%     Set = [];
%     
%     while 1
%         dl = regexp( line, ';', 'split' );
%         
%         gc = str2double(dl(2));
%         p = str2double(dl(4));
%         tp = str2double(dl(5));
%         fp = str2double(dl(6));
%         fn = str2double(dl(7));
%         
%         Set = [Set; [gc,p,tp,fp,fn]];
%         
%         line = fgetl(file);
%         if ~ischar(line), break, end
%     end
%     fclose(file);
%     
    %size(Set)
    
    path


end

