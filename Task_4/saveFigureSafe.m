%% Save a figure as PNG without ever stopping the experiment
% exportgraphics can fail on figures with many semi-transparent robot
% meshes and lighting (MATLAB's Java raster exporter throws a
% NullPointerException), or when the figure window was closed while the
% script was running. Saving a picture should never cost the planning
% results, so this tries three exporters in turn and only warns if all fail.
function ok = saveFigureSafe(fig, fileName)

ok = false;
if isempty(fig) || ~isvalid(fig)
    warning('saveFigureSafe: figure was closed before saving %s (skipped).', fileName);
    return;
end
drawnow;   % make sure the figure is fully rendered before exporting

try
    exportgraphics(fig, fileName, 'Resolution', 150);
    ok = true; return;
catch
end
try
    print(fig, fileName, '-dpng', '-r150', '-opengl');
    ok = true; return;
catch
end
try
    frame = getframe(fig);            % screen capture of the figure window
    imwrite(frame.cdata, fileName);
    ok = true; return;
catch err
    warning('saveFigureSafe: could not save %s (%s). Continuing.', fileName, err.message);
end
end
