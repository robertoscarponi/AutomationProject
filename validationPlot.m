function validationPlot(x, y1, y2, x_label, y1_label, y2_label, testinput)
    figure
    plot(x, y1, "Color", [46/255, 134/255, 193/255], "LineWidth", 2.5);
    xlabel(x_label);
    ylabel(y1_label);
    hold on
    plot(x, y2, "Color", [1, 85/255, 51/255], "LineWidth", 2.5);
    xlabel(x_label);
    ylabel(y2_label);
    title({
        "Comparison Between the Real Response and the Estimated Response - ", ...
        testinput
    });
    legend("Real sistem", "Estimated Sistem")

end