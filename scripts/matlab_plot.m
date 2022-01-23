database_dir = "../../AreaCoverage-dataset/"
database_name = "RAL_main"

figure
tiledlayout(2,2)
nexttile
fid = fopen(database_dir + database_name + "/env_data",'r');
if fid == -1
  error('matlab_plot:OpenFile', 'Cannot open file: %s', database_dir + database_name + "/env_data",'r');
end
line_ex = fgetl(fid);
flag = 0;
hold on
pgon_all = [];
colorOrder = get(gca, 'ColorOrder');
colorOrder = [colorOrder; colorOrder];
while ischar(line_ex)
    x = str2num(line_ex);
    line_ex = fgetl(fid);
    y = str2num(line_ex);
    line_ex = fgetl(fid);
    pgon = polyshape(x, y);
		pgon_all = [pgon_all, pgon];
    if flag == 0
        plot(pgon,'FaceColor','white','FaceAlpha',0.5, 'LineWidth', 1.2);
        flag = 1;
    else
        plot(pgon,'FaceColor','#cacaca', 'FaceAlpha' , 1, 'LineWidth', 1.2);
    end
end
axis equal
x_limits = xlim;
y_limits = ylim;
del_x = abs(x_limits(2) - x_limits(1));
del_y = abs(y_limits(2) - y_limits(1));
axis_buffer = 0.00;
x_final_limits = x_limits + [floor(-axis_buffer * del_x), ceil(axis_buffer * del_x)];
y_final_limits = y_limits + [floor(-axis_buffer * del_y), ceil(axis_buffer * del_y)];
xlim(x_final_limits);
ylim(y_final_limits);
title('Input Environment')
fclose(fid);

nexttile
hold on
plot(pgon_all(1),'FaceColor','white','FaceAlpha',0.5, 'LineWidth', 1.5);
for i=2:length(pgon_all)
	pgon = pgon_all(i);
	plot(pgon,'FaceColor','#cacaca', 'FaceAlpha' , 1, 'LineWidth', 1.5);
end
fid = fopen(database_dir + database_name + "/mem/init_decomp",'r');
line_ex = fgetl(fid);
flag = 0;
i = 1;
while ischar(line_ex)
    x = str2num(line_ex);
    line_ex = fgetl(fid);
    y = str2num(line_ex);
    line_ex = fgetl(fid);
    pgon = polyshape(x, y);
    plot(pgon,'FaceColor', colorOrder(mod(i,7) + 1,:),'FaceAlpha',0.5);
    i = i + 1;
end
axis equal
xlim(x_final_limits);
ylim(y_final_limits);
title('Initial Decomposition')
fclose(fid);


nexttile
hold on

fid = fopen(database_dir + database_name + "/mem/final_decomp",'r');
line_ex = fgetl(fid);
flag = 0;
i = 1;
while ischar(line_ex)
    x = str2num(line_ex);
    line_ex = fgetl(fid);
    y = str2num(line_ex);
    line_ex = fgetl(fid);
    pgon = polyshape(x, y);
    plot(pgon,'FaceColor', colorOrder(mod(i,7) + 1,:),'FaceAlpha',0.5);
    i = i + 1;
end
axis equal
xlim(x_final_limits);
ylim(y_final_limits);
title('Final Decomposition')
fclose(fid);
plot(pgon_all(1),'FaceColor','white','FaceAlpha',0.5, 'LineWidth', 1.2);
for i=2:length(pgon_all)
	pgon = pgon_all(i);
	plot(pgon,'FaceColor','#cacaca', 'FaceAlpha' , 1, 'LineWidth', 1.2);
end
nexttile
hold on

fid = fopen(database_dir + database_name + "/mem/tracks",'r');
line_ex = fgetl(fid);
i = 1;
color = colorOrder(mod(i, 7) + 1, :);
while (~feof(fid))
	if(size(line_ex) == 0)
		i = i + 1;
		color = colorOrder(mod(i, 7) + 1, :);
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
xlim(x_final_limits);
ylim(y_final_limits);

fid = fopen(database_dir + database_name + "/mem/final_decomp",'r');
line_ex = fgetl(fid);
i = 1;
while ischar(line_ex)
	x = str2num(line_ex);
	line_ex = fgetl(fid);
	y = str2num(line_ex);
	line_ex = fgetl(fid);
	pgon = polyshape(x, y);
	plot(pgon,'FaceColor', colorOrder(mod(i,7) + 1,:),'FaceAlpha',0.2, 'LineWidth', 0.8);
	i = i + 1;
end
fclose(fid);

plot(pgon_all(1),'FaceColor','white','FaceAlpha',0.5, 'LineWidth', 1.2);
for i=2:length(pgon_all)
	pgon = pgon_all(i);
	plot(pgon,'FaceColor','#cacaca', 'FaceAlpha' , 1, 'LineWidth', 1.2);
end
title('Final Tracks')

saveas(gcf,database_dir + database_name + "/results.png")
