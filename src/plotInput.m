
function plotInput(x, y, plot_title, xlabel_text, ylabel_text)
    plot(x, y, 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5);
    grid on
    zoom on
    box off
    title(plot_title)
    xlabel(xlabel_text);
    ylabel(ylabel_text);
end
