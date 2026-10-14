import json
import struct
import os
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- 配置 ---
JSON_FILE = 'rt_trace_layout.json'
REFRESH_INTERVAL = 20   # ms
MAX_POINTS = 100000     # 窗口保留多少个点

# 全局数据存储
traces = {}
sys_period_ms = 20.0

def load_layout():
    """加载配置并初始化数据结构"""
    global sys_period_ms
    if not os.path.exists(JSON_FILE):
        return False
    
    try:
        with open(JSON_FILE, 'r') as f:
            layout = json.load(f)
        
        sys_period_ms = layout.get('period_ms', 20.0)
        signals = layout.get('signals', [])
        
        for item in signals:
            name = item['name']
            
            # 类型映射
            val_fmt = {'float': 'f', 'double': 'd', 'int32': 'i'}.get(item['type'], 'f')
            
            # 协议: [Tick(4B)] + [Value(N B)]
            pkt_fmt = '<I' + val_fmt
            pkt_size = 4 + item['size'] 
            
            traces[name] = {
                'filename': item['filename'],
                'pkt_size': pkt_size,
                'pkt_fmt': pkt_fmt,
                'x_data': [], 
                'y_data': [], 
                'file_handle': None,
                'line_obj': None,
                'leftover': b'' # 【新增】用于缓存未凑齐一个包的残余字节
            }
        return True
    except Exception as e:
        print(f"[WARN] JSON Error: {e}")
        return False

def process_file_data(info):
    """
    核心解析函数：读取文件新增内容并解析
    返回: True如有新数据, False如无
    """
    # 1. 确保文件已打开
    if info['file_handle'] is None:
        if os.path.exists(info['filename']):
            # 'rb' 模式打开，文件指针默认在开头，可以读取历史数据
            info['file_handle'] = open(info['filename'], 'rb')
        else:
            return False

    fh = info['file_handle']
    size = info['pkt_size']
    fmt = info['pkt_fmt']
    
    # 2. 读取所有新数据
    new_bytes = fh.read()
    if not new_bytes:
        return False

    # 3. 【残包处理】拼接上次剩余的字节
    raw_data = info['leftover'] + new_bytes
    total_len = len(raw_data)
    
    # 4. 计算完整包的数量
    count = total_len // size
    remainder = total_len % size
    
    if count > 0:
        # 批量处理完整包 (切片操作)
        # 注意：如果数据量巨大，这里可以进一步优化，但当前逻辑足够
        valid_data = raw_data[:count*size]
        
        for i in range(count):
            chunk = valid_data[i*size : (i+1)*size]
            tick, val = struct.unpack(fmt, chunk)
            
            t_sec = (tick * sys_period_ms) / 1000.0
            info['x_data'].append(t_sec)
            info['y_data'].append(val)
        
        # 裁剪窗口
        if len(info['x_data']) > MAX_POINTS:
            info['x_data'] = info['x_data'][-MAX_POINTS:]
            info['y_data'] = info['y_data'][-MAX_POINTS:]
            
    # 5. 保存剩余不足一个包的字节供下次使用
    info['leftover'] = raw_data[count*size:]
    
    return count > 0

def preload_history():
    """【新增】在显示窗口前，先读取所有历史数据"""
    print("Pre-loading history data...")
    has_data = False
    for name, info in traces.items():
        if process_file_data(info):
            has_data = True
            print(f"  - {name}: Loaded {len(info['x_data'])} points.")
    
    if not has_data:
        print("  - No history data found.")

def init_plot():
    fig, ax = plt.subplots()
    ax.set_title(f"Monitor (Period: {sys_period_ms}ms)")
    ax.set_xlabel("Time (s)")
    ax.grid(True)
    
    # 绘制初始线条（包含刚刚预加载的数据）
    all_x = []
    all_y = []
    
    for name, info in traces.items():
        # 如果有预加载数据，这里直接画上去
        line, = ax.plot(info['x_data'], info['y_data'], label=name)
        info['line_obj'] = line
        
        if info['x_data']:
            all_x.extend(info['x_data'])
            all_y.extend(info['y_data'])

    # 初始视口调整
    if all_x and all_y:
        ax.set_xlim(min(all_x), max(all_x) + 0.1)
        y_min, y_max = min(all_y), max(all_y)
        margin = (y_max - y_min) * 0.1 if y_max != y_min else 1.0
        ax.set_ylim(y_min - margin, y_max + margin)

    ax.legend(loc='upper left')
    return fig, ax

def update(frame):
    """动画回调：只负责读取增量数据和刷新UI"""
    has_update = False
    
    # 遍历所有通道读取数据
    for info in traces.values():
        if process_file_data(info):
            # 只有当数据真的变了，才去更新线条对象
            info['line_obj'].set_data(info['x_data'], info['y_data'])
            has_update = True

    # 自动缩放坐标轴
    if has_update:
        ax = traces[list(traces.keys())[0]]['line_obj'].axes
        all_x = []
        all_y = []
        for info in traces.values():
            all_x.extend(info['x_data'])
            all_y.extend(info['y_data'])
        
        if all_x:
            ax.set_xlim(min(all_x), max(all_x) + 0.1)
            # Y轴也可以自动缩放，但如果数据跳变太快可能眼花，可视情况注释掉
            y_min, y_max = min(all_y), max(all_y)
            margin = (y_max - y_min) * 0.1 if y_max != y_min else 1.0
            ax.set_ylim(y_min - margin, y_max + margin)

    return [info['line_obj'] for info in traces.values()]

# --- Main Flow ---
print("Waiting for simulation layout...")
while not load_layout():
    time.sleep(1)

# 1. 先同步历史数据 (防止第一帧卡死)
preload_history()

print("Starting Plotter...")
# 2. 初始化图表（此时线条上已经有历史数据了）
fig, ax = init_plot()

# 3. 开启增量更新
ani = animation.FuncAnimation(
    fig, 
    update, 
    interval=REFRESH_INTERVAL, 
    blit=False, 
    cache_frame_data=False
)

plt.show()

# 清理资源
for info in traces.values():
    if info['file_handle']:
        info['file_handle'].close()
        