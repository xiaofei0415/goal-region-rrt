# goal-region-rrt
如下图，给定一个规划问题，其中，黑色粗线是目标区域，从起始点到达该区域内任一点都是规划成功的，但显然目标点(56,80)是可以提供最短路径的，本仓库的规划算法可以在线筛选出一个可提供最短路径的目标点。

![image](https://user-images.githubusercontent.com/80371110/159273589-09ac3e2d-1e6f-4481-ba0f-ec5f2a9b5813.png)

下面四张图是路径收敛过程，“+”字号代表当前的候选目标点，绿色粗线代表当前最优路径，最终获得一个穿越狭缝的最短路径，相应的路径末端就是筛选出的最优目标点了，在较短的运行时间和最多四个候选目标点的参数设置下，该结果还是比较接近理论最优的。

对应复杂的障碍物，建议增加numgoals参数，即增加更多的候选目标点。

![image](https://user-images.githubusercontent.com/80371110/159273520-f9a23480-9110-4f48-87de-aa9a56942a99.png)

![image](https://user-images.githubusercontent.com/80371110/159273629-95ac5674-96a7-4400-88e5-ecd3a9feadb2.png)

![image](https://user-images.githubusercontent.com/80371110/159273657-8d626cb1-102c-46e7-978e-96685f7a6dda.png)

![image](https://user-images.githubusercontent.com/80371110/159273673-4febe510-35e7-4051-a719-390776027c44.png)

算法还不够完善，对初始候选点比较敏感，我是随机生成初始候选目标点的，最终的最优路径会有一些随机性，感兴趣的可以帮助修改下。
