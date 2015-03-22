function [ output_args ] = plotResults( results, color, width,style)
    
    if (nargin < 4)
        style='-';
    end
    pr = results(:,8:9);
    
    plot(pr(:,1),pr(:,2),style, 'Color',color,'LineWidth',width);
    
end

