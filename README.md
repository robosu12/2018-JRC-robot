# 2018-JRC-robot
2018京东机器人挑战赛-机器人全自主导航

1. 背景说明：此次比赛为无人超市主题，在无人超市中放置许多货架，货架每层都放置许多物品（可乐，牙膏，牙刷，火腿肠，薯片，书本，口红等），物品清单已知，机器人在超市入口处待命。现场随机指定10种物品，然后一键启动机器人，要求机器人全自主运行，获取所有物品，然后放置到指定出口箱内。比赛时间5分钟，以最后抓取物品个数和完成时间计算分数。
2. 软件环境：Ubuntu16.04 + ROS kinetic
3. 功能说明：移动平台为四轮麦克纳姆轮全向移动平台，采用cartographer 2D 激光SLAM算法进行机器人定位和地图构建，catkin_ws_base 为移动平台核心控制节点，负责接收SLAM节点结果，接收抓取流程规划节点结果，进行障碍物躲避，扇形区域法局部路径规划，A-star 全局路径规划，定位误差补偿，放置区域识别等任务，然后和移动平台STM32F4嵌入式下位机通过串口进行通信，控制平台移动。
