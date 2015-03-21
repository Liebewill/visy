function [ output_args ] = plotResults( results, color, width)
    
    pr = results(:,8:9);
    
    plot(pr(:,1),pr(:,2),'Color',color,'LineWidth',width);
    
end

