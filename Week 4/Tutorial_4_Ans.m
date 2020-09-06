

bag = rosbag('carto_cas_exp1.bag');
laserSelect = select(bag, 'Topic', '/scan');
firstlaserscan = readMessages(laserSelect, 1);
cart = firstlaserscan{1}.readCartesian;

plot(cart(:,1),cart(:,2))