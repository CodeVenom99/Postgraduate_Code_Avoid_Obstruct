# Postgraduate_Code_Avoid_Obstruct

避障
obstruct_num		探测障碍个数

CFM			人工势场							v1.0
CFM2			对于墙的避障行为迟缓，将斥力向引力方向旋转90度后计算合力
CFM3			CFM_2在避障过程中存在双抖动路线
WFM			沿墙算法，就是将墙的斥力旋转90度作为合力，忽略目标点引力		v2.0

reach_key_point		添加到达关键点调整姿态函数					v2.1
WFM_pro			车速不再与斥引力大小有关，而是固定车速				v2.2
WFM_pro_2		1.WFM的车速设置为 WFM_speed*(1-fabs(cos(Repulsion_Force.rad)));
			2.斥力分为根据CFM或WFM模式切换					v2.3
high_oscillatory	想让在cfm模式下轨迹更加的平滑，试用角度底通滤波器没有效果,
			可能是pid调节原因？						v2.4
WFM_pid			想通过pid调节使得控制更加的敏锐，失败				v2.5
stable_switch		针对cfm和wfm状态切换不稳定，增加消抖的代码并做了一些测试，			
			发现引力系数k和转向力系数TURN_COEFFICIENT会影响cFM的抖动效果	v2.6