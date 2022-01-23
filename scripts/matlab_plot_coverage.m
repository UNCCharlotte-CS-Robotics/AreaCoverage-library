database_dir = "../../AreaCoverage-dataset/"
database_name = "RAL_main"
fid = fopen(database_dir + database_name + "/mem/coverage",'r');
line_ex = fgetl(fid);
flag = 0;
figure
hold on
colorOrder = get(gca, 'ColorOrder');
colorOrder = [colorOrder; colorOrder];
i = 1;
while ischar(line_ex)
    x = str2num(line_ex);
    line_ex = fgetl(fid);
    y = str2num(line_ex);
    line_ex = fgetl(fid);
    pgon = polyshape(x, y);
		plot(pgon,'FaceColor','blue','FaceAlpha',0.5, 'LineWidth', 1, 'LineStyle', 'none');
    i = i + 1;
end
fclose(fid);
fid = fopen(database_dir + database_name + "/env_data",'r');
line_ex = fgetl(fid);
flag = 0;
hold on
pgon_all = [];
while ischar(line_ex)
    x = str2num(line_ex);
    line_ex = fgetl(fid);
    y = str2num(line_ex);
    line_ex = fgetl(fid);
    pgon = polyshape(x, y);
		pgon_all = [pgon_all, pgon];
    if flag == 0
        plot(pgon,'FaceColor','white','FaceAlpha',0, 'LineWidth', 1.2);
        flag = 1;
    else
        plot(pgon,'FaceColor','black','FaceAlpha',0.5, 'LineWidth', 1.2);
    end
end
fclose(fid);
fid = fopen(database_dir + database_name + "/mem/tracks",'r');
line_ex = fgetl(fid);
i = 1;
color = colorOrder(mod(i, 7) + 0, :);
while (~feof(fid))
	if(size(line_ex) == 0)
		line_ex = fgetl(fid);
	else
		p1 = str2num(line_ex);
		line_ex = fgetl(fid);
		p2 = str2num(line_ex);
		line_ex = fgetl(fid);
		line([p1(1), p2(1)], [p1(2), p2(2)], 'Color', color);
	end
end
fclose(fid);
axis equal
x_limits = xlim;
y_limits = ylim;
del_x = abs(x_limits(2) - x_limits(1));
del_y = abs(y_limits(2) - y_limits(1));
xlim(x_limits + [floor(- 0.2 * del_x), ceil(0.2 * del_x)]);
ylim(y_limits + [floor(- 0.2 * del_y), ceil(0.2 * del_y)]);

saveas(gcf,database_dir + database_name + "/coverage.png")
