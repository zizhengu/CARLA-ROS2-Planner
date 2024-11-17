import carla
import random

# 客户端连接并获取时间对象
client = carla.Client('172.31.192.1', 2000)
world = client.get_world()
world.unload_map_layer(carla.MapLayer.Buildings)
settings = world.get_settings()
settings.synchronous_mode = False # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(False)

# 如果有必要设置一个种子，以便能够重现行为
traffic_manager.set_random_device_seed(0)
random.seed(0)

spectator = world.get_spectator()
spawn_points = world.get_map().get_spawn_points()
print("----------------")
print(spawn_points[0:5])
print("----------------")
##[<carla.libcarla.Transform object at 0x7fe66a7a85f0>, <carla.libcarla.Transform object at 0x7fe66a7a8740>, <carla.libcarla.Transform object at 0x7fe66a7a87b0>, <carla.libcarla.Transform object at 0x7fe66a7a8820>, <carla.libcarla.Transform object at 0x7fe66a7a8890>]
# spawn_point_1 = carla.Transform(
#         carla.Location(x=-197.405151, y=-169.926147, z=0.5),
#         carla.Rotation(pitch=0.061376, yaw=177.044647, roll=0.000000),
#     )
    
# 在地图上以数字绘制生成点的位置
# for i, spawn_point in enumerate(spawn_points):
#     world.debug.draw_string(spawn_point.location, str(i), life_time=10)

# 从蓝图库中选择一些模型
models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
blueprints = []
for vehicle in world.get_blueprint_library().filter('*vehicle*'):
    if any(model in vehicle.id for model in models):
        blueprints.append(vehicle)

# 路线 1
spawn_point_1 =  spawn_points[32]
# 从所选择的生成点中创建路线 1
route_1_indices = [129, 28, 124, 33, 97, 119, 58, 154, 147]
route_1 = []
for ind in route_1_indices:
    route_1.append(spawn_points[ind].location)

# 路线 2
spawn_point_2 =  spawn_points[149]
# 从所选择的生成点中创建路线 2
route_2_indices = [21, 76, 38, 34, 90, 3]
route_2 = []
for ind in route_2_indices:
    route_2.append(spawn_points[ind].location)

# 现在让我们在地图上打印出它们，以便我们能看到我们的路径
world.debug.draw_string(spawn_point_1.location, 'Spawn point 1', life_time=600, color=carla.Color(255,0,0))
world.debug.draw_string(spawn_point_2.location, 'Spawn point 2', life_time=600, color=carla.Color(0,0,255))

for ind in route_1_indices:
    spawn_points[ind].location
    world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=600, color=carla.Color(255,0,0))
    
for ind in route_2_indices:
    spawn_points[ind].location
    world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=600, color=carla.Color(0,0,255))
    

# 在生成时间之间设置延迟以创建空隙
spawn_delay = 20
counter = spawn_delay

# 设置最大车辆（为更低的硬件设置更小的值）
max_vehicles = 20
# 在生成点之间轮流
alt = False
while True:
    world.tick()

    n_vehicles = len(world.get_actors().filter('*vehicle*'))
    vehicle_bp = random.choice(blueprints)

    # 仅在延迟之后生成车辆
    if counter == spawn_delay and n_vehicles < max_vehicles:
        # Alternate spawn points
        if alt:
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point_1)
            
        else:
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point_2)

        if vehicle: # 如果车辆成功生成
            vehicle.set_autopilot(True) # 将车辆的控制全交给交通管理器
            # 设置交通管理器车辆控制的参数，我们不想变道
            traffic_manager.update_vehicle_lights(vehicle, True)
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)

            # 在生成点之间轮流
            if alt:
                traffic_manager.set_path(vehicle, route_1)
                alt = False
            else:
                traffic_manager.set_path(vehicle, route_2)
                alt = True

            vehicle = None
        counter -= 1
    elif counter > 0:
        counter -= 1
    elif counter == 0:
        counter = spawn_delay



# # 设置车辆的最大数目并准备我们生成的一个列表
# max_vehicles = 20
# max_vehicles = min([max_vehicles, len(spawn_points)])
# vehicles = []

# # 对生成点进行随机采样并生成一些车辆。
# for i, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
#     temp = world.try_spawn_actor(random.choice(blueprints), spawn_point)
#     if temp is not None:
#         vehicles.append(temp)
#         print(f"Spawned vehicle: {temp.type_id} at {spawn_point.location}")
#     else:
#         print("Failed to spawn vehicle.")
# # 解析所生成车辆的列表并通过set_autopilot()将控制权交给交通管理器
# for vehicle in vehicles:
#     vehicle.set_autopilot(True)
#     # Randomly set the probability that a vehicle will ignore traffic lights
#     traffic_manager.ignore_lights_percentage(vehicle, random.randint(0,50))

    
