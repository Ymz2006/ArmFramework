import socket
import json
import time
import argparse
import struct
from typing import Tuple, Optional, Any
import pid_controller as pid_controller

PI = 3.1415926

def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return (True, sock)
    except Exception as e:
        sock.close()
        print(e)
        return (False,)

def disconnectETController(sock):
    if sock:
        sock.close()
        sock = None
    else:
        sock = None

def sendCMD(sock, cmd, params=None, id=1):
    if not params:
        params = []
    else:
        params = json.dumps(params)
    sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(cmd, params, id) + "\n"
    try:
        sock.sendall(bytes(sendStr, "utf-8"))
        ret = sock.recv(1024)
        jdata = json.loads(str(ret, "utf-8"))
        if "result" in jdata.keys():
            return (True, json.loads(jdata["result"]), jdata["id"])
        elif "error" in jdata.keys():
            return (False, jdata["error"], jdata["id"])
        else:
            return (False, None, None)
    except Exception as e:
        return (False, None, None)


class ECRobot():
    def __init__(self, name,host=None) -> None:
        # Parse arguments
        self.robot_ip = "192.168.1.20"
        self.conSuc, self.sock = connectETController(self.robot_ip)
        self.agv_ip = "192.168.192.5"
        self.agv_conSuc, self.agv_sock = connectETController(self.agv_ip, port=19206)
        self.agv_status_conSuc, self.agv_status_sock = connectETController(self.agv_ip, port=19204)

    def check_ready(self):
        return True
    
    def read_current_pose(self):
        '''
            output:
                pose: xyz rxryrz
        '''
        # feedback = self.base_cyclic.RefreshFeedback()
        # x,y,z = feedback.base.tool_pose_x,feedback.base.tool_pose_y,feedback.base.tool_pose_z
        # rx,ry,rz = feedback.base.tool_pose_theta_x/360*2*PI,feedback.base.tool_pose_theta_y/360*2*PI,feedback.base.tool_pose_theta_z/360*2*PI
        
        if self.conSuc:
            suc, result , id=sendCMD(self.sock,"get_tcp_pose")
            # print(f"{result}")
            result[0] /= 1000
            result[1] /= 1000
            result[2] /= 1000

            return result
    
    def set_pose(self,pose, is_degree=True):
        '''
            input:
                pose: xyz rxryrz
        '''
        pose[0] *= 1000
        pose[1] *= 1000
        pose[2] *= 1000
        print("Starting Cartesian action movement ...")
        if self.conSuc:
            # 开伺服
            suc, result, id = sendCMD(self.sock, "set_servo_status", {"status":1})
            print(result)
            # time.sleep(1)

            print("Executing action")
            suc, result , id=sendCMD(self.sock,"moveByLineCoord",{"targetUserPose": pose,"user_coord":[0,0,0,0,0,0], "speed_type" :0, "speed":300,"unit_type":1})
            # 打印结果
            print(f"end:{result}")

            print("Waiting for movement to finish ...")
            while True:
                suc, result , id=sendCMD(self.sock,"getRobotState")
                if result == 0:
                    break
            # 关伺服
            suc, result, id = sendCMD(self.sock, "set_servo_status", {"status":0})
            # print(result)
            if result:
                print("Cartesian movement completed")
            else:
                print("Timeout on action notification wait")
            return result
    
    def set_hand(self, hand_state):
        print("Starting hand control...")
        if self.conSuc:
            # 开伺服
            suc, result, id = sendCMD(self.sock, "set_servo_status", {"status":1})
            print(result)

            print("Executing hand control")
            if hand_state == "open":
                suc, result18, id=sendCMD(self.sock,"setOutput",{"addr":18,"status":0})
                suc, result19, id=sendCMD(self.sock,"setOutput",{"addr":19,"status":1})
                time.sleep(1.5)
                suc, result, id=sendCMD(self.sock,"setOutput",{"addr":19,"status":0})
            elif hand_state == "close":
                suc, result19, id=sendCMD(self.sock,"setOutput",{"addr":19,"status":0})
                suc, result18, id=sendCMD(self.sock,"setOutput",{"addr":18,"status":1})
                time.sleep(1.5)
                suc, result, id=sendCMD(self.sock,"setOutput",{"addr":18,"status":0})
            else:
                print("Invalid hand_state")
                return False
            
            suc, result, id = sendCMD(self.sock, "set_servo_status", {"status":0})
            
            if result18 and result19:
                print("hand control completed")
            else:
                print("Timeout on hand control notification wait")
            return result
        
    def get_hand(self, io_id):
        print("Starting hand control...")
        if self.conSuc:
            print("Executing hand control")
            suc, result, id=sendCMD(self.sock,"getOutput",{"addr":io_id})
            print(f"IO_{io_id}={result}")
            
            return True


    def send_agv_command(self, sock, cmd_type, data, seq=1):
        """发送AGV协议命令并接收响应"""
        try:
            # 构造协议头
            sync = 0x5A
            version = 1
            number = seq
            json_data = json.dumps(data).encode('utf-8')
            length = len(json_data)
            reserved = bytes(6)  # 保留区域填充0
            
            # 大端序打包头部
            header = struct.pack(
                '>BBHIH6s', 
                sync, version, number, length, cmd_type, reserved
            )
            
            # 发送数据
            sock.sendall(header + json_data)
            
            # 接收响应头
            response_header = sock.recv(16)
            if len(response_header) < 16:
                return (False, None)
            
            # 解析响应头
            sync_res, version_res, number_res, length_res, cmd_type_res, _ = struct.unpack(
                '>BBHIH6s', response_header
            )
            if sync_res != 0x5A or version_res != 1:
                return (False, None)
            
            # 读取数据区
            data_res = b''
            if length_res > 0:
                data_res = sock.recv(length_res)
                while len(data_res) < length_res:
                    data_res += sock.recv(length_res - len(data_res))
            
            # 解析JSON响应
            response = json.loads(data_res.decode('utf-8', errors='ignore'))
            if response.get('ret_code', 0) != 0:
                return (False, response)
            return (True, response)
        
        except Exception as e:
            print(f"AGV通信异常: {str(e)}")
            return (False, None)

    def set_agv(self, waypoint):
        """发送固定路径导航指令并监控状态"""
        if not (self.agv_conSuc and self.agv_status_conSuc):
            print("AGV连接异常")
            return False
        
        # 1. 发送导航指令（假设报文类型3001）
        nav_data = {
            "source_id": "",  # 需根据实际情况调整源站点
            "id": waypoint           # 目标站点
        }
        success, resp = self.send_agv_command(self.agv_sock, 3051, nav_data, seq=1)
        print(f"回复: {success},{resp}")
        if not success:
            print(f"导航指令失败: {resp.get('err_msg', '未知错误')}")
            return False
        
        # 2. 轮询导航状态（报文类型1020）
        seq = 2  # 序号递增
        while True:
            success, resp = self.send_agv_command(
                self.agv_status_sock, 1020, {"simple": True}, seq=seq
            )
            seq += 1
            
            if not success:
                print(f"状态查询失败，重试中...")
                time.sleep(1)
                continue
            
            # 解析状态
            status = resp.get('task_status', -1)
            # task_type = resp.get('task_type', -1)
            # target_id = resp.get('target_id', -1)
            # ret_code = resp.get('ret_code', -1)
            # err_msg = resp.get('err_msg', -1)
            # print(f"状态: {status},{task_type},{target_id},{ret_code},{err_msg}")
            if status == 4:   # COMPLETED
                print("AGV导航完成")
                return True
            elif status == 5:  # FAILED
                print("AGV导航失败")
                return False
            elif status == 2: # RUNNING
                print(f"导航前往{waypoint}进行中...")
            
            time.sleep(1)  # 间隔1秒查询
    def get_agv_status(self) -> Tuple[bool, dict]:
        """
        获取 AGV 的状态信息

        :return: (成功标志, 状态数据或错误信息)
        """
        cmd_type = 1100      # 报文类型，整数
        expected_response_type = 11004  # 期望的响应类型，整数
        request_data = {
            "request": "status"
        }
        seq = 1  # 序列号，从1开始

        try:
            # 发送命令
            print(f"发送获取 AGV 状态的命令: {request_data}, 报文类型: {cmd_type}, 序列号: {seq}")
            success, response = self.send_agv_command(self.agv_status_sock, cmd_type, request_data, seq=seq)
            
            if not success:
                print(f"发送命令失败: {response}")
                return (False, response)
            
            # 打印接收到的原始响应数据（用于调试）
            # if isinstance(response, bytes):
            #     print(f"接收到原始响应数据（十六进制）: {response.hex()}")
            # else:
            #     print(f"接收到响应数据: {response}")

            # 检查响应类型是否匹配期望值
            # 注意：send_agv_command 已经检查了 cmd_type_res 是否等于 cmd_type
            # 这里可以根据需要进一步验证响应内容
            if isinstance(response, dict):
                # 可选：打印完整的响应数据
                # print(f"接收到 AGV 状态数据: {json.dumps(response, indent=4, ensure_ascii=False)}")
                return (True, response)
            else:
                print(f"收到无效的响应数据: {response}")
                return (False, {"ret_code": 60002, "err_msg": "无效的响应数据"})
        
        except Exception as e:
            print(f"获取 AGV 状态时发生异常: {e}")
            return (False, {"ret_code": 60002, "err_msg": f"异常: {e}"})

    def close(self):
        # self.router.__exit__()
        if self.sock: self.sock.close()
        if self.agv_sock: self.agv_sock.close()
        if self.agv_status_sock: self.agv_status_sock.close()


if __name__ == '__main__':
    robot = ECRobot("ec_robot")
    
    curr_pos = robot.read_current_pose()
    print(curr_pos)
    robot.set_pose([0.1, 0, 0.5, 0, 0, 0])
    curr_pos = robot.read_current_pose()
    print(curr_pos)





