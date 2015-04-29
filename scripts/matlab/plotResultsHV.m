function [ output_args ] = plotResultsHV( results, color, width,style)
    
    if (nargin < 4)
        style='-';
    end
    pr = results(:,16:17);
    
    plot(pr(:,1),pr(:,2),style, 'Color',color,'LineWidth',width);
    
end

