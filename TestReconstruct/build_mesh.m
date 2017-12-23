heights = csvread('output/key_frame_heights_0.txt');

[el,i] = min(heights(:,3));
heights(i,:)=[]; % это мы удалили наименьший элемент

[xq,yq] = meshgrid(-100:0.9:100, -100:0.9:100);
vq = griddata(heights(:,1), heights(:,2), heights(:,3), xq, yq);

figure
mesh(xq, yq, vq, 'EdgeColor', [0.5 0.5 0.5], 'FaceColor', 'interp')
hold on
plot3(heights(:,1), heights(:,2), heights(:,3), '.', 'Color', 'b')

axis ij
xlabel('X (m)') % x-axis label
ylabel('Y (m)') % y-axis label

xlim([-40 40])
ylim([-40 40])

colormap jet