import serial
import serial.tools.list_ports
import time
import random
import string
import argparse
import sys
from collections import deque

def find_serial_ports():
    """返回系统上可用的串口列表"""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def generate_random_string(length):
    """生成指定长度的随机字符串（包含可打印ASCII字符）"""
    return ''.join(random.choices(string.ascii_letters + string.digits + string.punctuation, k=length))

def format_data(data, max_display=50):
    """格式化数据用于显示，控制最大显示长度"""
    if len(data) <= max_display:
        return f"'{data}'"
    else:
        return f"'{data[:max_display//2]}...{data[-max_display//2:]}' ({len(data)} chars)"

def test_bidirectional_communication(port1, port2, baudrate, test_size, timeout=10):
    """
    测试双向串口通信的可靠性
    :param port1: 串口1名称
    :param port2: 串口2名称
    :param baudrate: 波特率
    :param test_size: 测试数据大小（字符数）
    :param timeout: 超时时间（秒）
    :return: 测试结果字典
    """
    # 生成随机测试字符串
    test_string1 = generate_random_string(test_size)
    test_string2 = generate_random_string(test_size)
    
    # 配置串口参数
    serial_params = {
        'baudrate': baudrate,
        'bytesize': serial.EIGHTBITS,
        'parity': serial.PARITY_NONE,
        'stopbits': serial.STOPBITS_ONE,
        'timeout': timeout,
        'write_timeout': timeout
    }
    
    results = {
        'baudrate': baudrate,
        'test_size': test_size,
        'success': False,
        'port1_send_success': False,
        'port2_send_success': False,
        'port1_received': "",
        'port2_received': "",
        'error': ""
    }
    
    try:
        with serial.Serial(port1, **serial_params) as ser1, \
             serial.Serial(port2, **serial_params) as ser2:
            
            # 清空缓冲区
            ser1.reset_input_buffer()
            ser2.reset_input_buffer()
            
            # 创建接收缓冲区
            port1_received = bytearray()
            port2_received = bytearray()
            
            # 启动接收线程
            receive_complete = threading.Event()
            
            def receiver(serial_port, buffer, port_name):
                while not receive_complete.is_set():
                    if serial_port.in_waiting:
                        chunk = serial_port.read(serial_port.in_waiting)
                        buffer.extend(chunk)
                        # 实时显示接收到的数据
                        print(f"  {port_name} received: {format_data(chunk.decode('ascii', 'ignore'))}")
            
            # 启动两个接收线程
            thread1 = threading.Thread(target=receiver, args=(ser1, port1_received, port1))
            thread2 = threading.Thread(target=receiver, args=(ser2, port2_received, port2))
            
            thread1.daemon = True
            thread2.daemon = True
            
            thread1.start()
            thread2.start()
            
            # 显示发送信息
            print(f"  {port1} sending: {format_data(test_string1)}")
            print(f"  {port2} sending: {format_data(test_string2)}")
            
            # 双向同时发送
            ser1.write(test_string1.encode('ascii'))
            ser2.write(test_string2.encode('ascii'))
            
            # 等待传输完成
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if len(port1_received) >= len(test_string2) and len(port2_received) >= len(test_string1):
                    break
                time.sleep(0.1)
            
            # 停止接收线程
            receive_complete.set()
            thread1.join(0.5)
            thread2.join(0.5)
            
            # 解码接收到的数据
            port1_received_str = port1_received.decode('ascii', 'ignore')
            port2_received_str = port2_received.decode('ascii', 'ignore')
            
            # 检查数据完整性
            port1_success = port1_received_str == test_string2
            port2_success = port2_received_str == test_string1
            
            results.update({
                'port1_send_success': port2_success,
                'port2_send_success': port1_success,
                'port1_received': port1_received_str,
                'port2_received': port2_received_str,
                'success': port1_success and port2_success
            })
    
    except serial.SerialException as e:
        results['error'] = str(e)
        print(f"  串口错误: {e}")
    except Exception as e:
        results['error'] = str(e)
        print(f"  未知错误: {e}")
    
    return results

def find_max_reliable_size(port1, port2, baudrate, start_size=1, max_size=1000000, factor=2, timeout=10):
    """
    查找最大可靠传输大小
    :param port1: 串口1名称
    :param port2: 串口2名称
    :param baudrate: 波特率
    :param start_size: 起始测试大小
    :param max_size: 最大测试大小
    :param factor: 每次增加倍数
    :param timeout: 超时时间（秒）
    :return: 最大可靠传输大小
    """
    current_size = start_size
    max_success_size = 0
    test_results = []
    
    print(f"\n查找最大可靠传输大小 @ {baudrate} baud:")
    
    while current_size <= max_size:
        print(f"\n测试大小: {current_size} 字符")
        
        result = test_bidirectional_communication(port1, port2, baudrate, current_size, timeout)
        test_results.append(result)
        
        # if 'error' in result:
        #     print(f"  错误: {result['error']}")
        #     break
        
        # 显示测试结果详情
        port1_status = "成功" if result['port1_send_success'] else "失败"
        port2_status = "成功" if result['port2_send_success'] else "失败"
        
        print(f"  {port1}->{port2}: {port1_status} | {port2}->{port1}: {port2_status}")
        
        if not result['port1_send_success']:
            print(f"  {port1} 发送到 {port2} 数据不匹配:")
            print(f"    发送: {format_data(generate_random_string(current_size))}")
            print(f"    接收: {format_data(result['port2_received'])}")
        
        if not result['port2_send_success']:
            print(f"  {port2} 发送到 {port1} 数据不匹配:")
            print(f"    发送: {format_data(generate_random_string(current_size))}")
            print(f"    接收: {format_data(result['port1_received'])}")
        
        if result['success']:
            print(f"  双向通信成功!")
            max_success_size = current_size
            current_size = int(current_size * factor)
        else:
            print(f"  通信失败，停止增加大小")
            break
    
    return max_success_size, test_results

def main():
    parser = argparse.ArgumentParser(
        description='透传模块双向通信可靠性测试工具',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--port1', required=True, help='串口1名称')
    parser.add_argument('--port2', required=True, help='串口2名称')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='串口波特率')
    parser.add_argument('--start-size', type=int, default=1,
                        help='起始测试大小（字符数）')
    parser.add_argument('--max-size', type=int, default=1000000,
                        help='最大测试大小（字符数）')
    parser.add_argument('--factor', type=float, default=2.0,
                        help='每次测试大小增加倍数')
    parser.add_argument('--timeout', type=int, default=10,
                        help='每次测试超时时间（秒）')
    parser.add_argument('--list-ports', action='store_true', 
                        help='列出可用串口并退出')
    
    args = parser.parse_args()
    
    if args.list_ports:
        ports = find_serial_ports()
        print("可用串口:")
        for port in ports:
            print(f"  - {port}")
        return
    
    print(f"透传模块双向通信可靠性测试")
    print(f"  串口1: {args.port1}")
    print(f"  串口2: {args.port2}")
    print(f"  波特率: {args.baudrate}")
    print(f"  起始测试大小: {args.start_size} 字符")
    print(f"  最大测试大小: {args.max_size} 字符")
    print(f"  大小增加倍数: {args.factor}")
    print(f"  测试超时时间: {args.timeout} 秒\n")
    
    # 查找最大可靠传输大小
    max_size, test_results = find_max_reliable_size(
        args.port1,
        args.port2,
        args.baudrate,
        args.start_size,
        args.max_size,
        args.factor,
        args.timeout
    )
    
    # 最终报告
    print("\n\n=== 测试总结 ===")
    print(f"最大可靠传输大小: {max_size} 字符")
    
    if max_size > 0:
        # 计算最大可靠大小的数据传输时间
        char_time = (max_size * 10) / args.baudrate  # 每个字符约10位（包括起始位、停止位）
        print(f"理论传输时间: {char_time:.2f} 秒 (@ {args.baudrate} baud)")
        
        # 显示测试结果统计
        success_count = sum(1 for r in test_results if r.get('success', False))
        total_tests = len(test_results)
        print(f"测试次数: {total_tests} | 成功次数: {success_count} | 成功率: {success_count/total_tests*100:.1f}%")
        
        # 显示最后一次成功测试的详情
        last_success = next((r for r in reversed(test_results) if r.get('success', False)), None)
        if last_success:
            print("\n最后一次成功测试详情:")
            print(f"  测试大小: {last_success['test_size']} 字符")
            print(f"  {args.port1}->{args.port2}: 成功")
            print(f"  {args.port2}->{args.port1}: 成功")
            print(f"  {args.port1} 接收数据: {format_data(last_success['port1_received'])}")
            print(f"  {args.port2} 接收数据: {format_data(last_success['port2_received'])}")
    else:
        print("没有成功完成任何大小的测试")

if __name__ == "__main__":
    import threading
    main()