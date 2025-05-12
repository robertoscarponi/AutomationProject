function plotResponse(x, y, plot_title, xlabel_text, ylabel_text)
    plot(x, y, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5);
    grid on
    zoom on
    box off
    title(plot_title)
    xlabel(xlabel_text);
    ylabel(ylabel_text);
end
