import errno
import os
import sys
import re
import json
import time
import logging
import socket
import numpy as np
import colorama
import pyttsx3
colorama.init(autoreset=True)

logging.basicConfig(level=logging.INFO)


sys.path.append(os.path.dirname(__file__))
from demo_env import GraspingEnv

import threading
import cv2
import mss
import numpy as np
import win32gui
import win32con
import win32com.client
import pygetwindow as gw

def bring_window_to_front(window_title):
    def get_all_hwnd(hwnd, mouse):
        if (win32gui.IsWindow(hwnd) and
            win32gui.IsWindowEnabled(hwnd) and
            win32gui.IsWindowVisible(hwnd)):
            hwnd_map.update({hwnd: win32gui.GetWindowText(hwnd)})

    hwnd_map = {}
    win32gui.EnumWindows(get_all_hwnd, 0)

    for h, t in hwnd_map.items():
        if t:
            if t == window_title:
                # h 为想要放到最前面的窗口句柄
                win32gui.BringWindowToTop(h)
                shell = win32com.client.Dispatch("WScript.Shell")
                shell.SendKeys('%')
                win32gui.SetForegroundWindow(h)
                win32gui.ShowWindow(h, win32con.SW_RESTORE)
                return h
    return None

def record_window(window_title, output_file="window_video.avi", stop_event=None):
    hwnd = bring_window_to_front(window_title)
    if hwnd is None:
        print(f"Window with title '{window_title}' not found.")
        return

    window = gw.getWindowsWithTitle(window_title)[0]
    x, y, width, height = window.left, window.top, window.width, window.height

    sct = mss.mss()
    monitor = {"top": y, "left": x, "width": width, "height": height}

    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    out = cv2.VideoWriter(output_file, fourcc, 8.0, (width, height))

    frame_rate = 8.0
    frame_duration = 1.0 / frame_rate

    while not stop_event.is_set():
        start_time = time.time()

        img = sct.grab(monitor)
        frame = np.array(img)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        out.write(frame)

        elapsed_time = time.time() - start_time
        time_to_wait = frame_duration - elapsed_time
        if time_to_wait > 0:
            time.sleep(time_to_wait)

    out.release()
    cv2.destroyAllWindows()
    print(f"Video saved as {output_file}")

def text_to_speech(text):
    # 初始化语音引擎
    engine = pyttsx3.init()
    
    # 设置语速
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate - 50)
    
    # 设置音量
    volume = engine.getProperty('volume')
    engine.setProperty('volume', volume + 0.25)
    
    # 播放文本
    engine.say(text)
    engine.runAndWait()

def extract_python_code(content):
    """ Extract the python code from the input content.
    :param content: message contains the code from gpt's reply.
    :return(str): python code if the content is correct
    """
    code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
    code_blocks = code_block_regex.findall(content)
    if code_blocks:
        full_code = "\n".join(code_blocks)
        if full_code.startswith("python"):
            full_code = full_code[7:]
        return full_code
    else:
        return None


def execute_python_code(pri, code):
    """ Execute python code with the input content.
    :param pri(Class): class name in prompts.
    :param code(str): python method to call.
    """
    print("\n"'\033[34m'"Please wait while I run the code in Sim...")
    print("\033[34m""code:" + code)
    try:
        exec(code)
        print('\033[32m'"Done!\n")
        return True
    except Exception as e:
        logging.warning('\033[31m'"Found error while running the code: {}".format(e))
        return False


def write_goal_positions_to_json(env, filename):
    try:
        gripper_pos, _ = env.get_current_pose()
        gripper_state = env.get_gripper_status()
        red_goal = env.get_body_pos('red_block')
        blue_goal = env.get_body_pos('blue_block')
        green_goal = env.get_body_pos('green_block')
        yellow_goal = env.get_body_pos('yellow_block')

        gripper_pos_list = gripper_pos.tolist()
        red_goal_list = red_goal.tolist()
        blue_goal_list = blue_goal.tolist()
        green_goal_list = green_goal.tolist()
        yellow_goal_list = yellow_goal.tolist()

        goal_pos = {
            "gripper_pos": gripper_pos_list,
            "gripper_state": gripper_state,
            "red_goal": red_goal_list,
            "blue_goal": blue_goal_list,
            "green_goal": green_goal_list,
            "yellow_goal": yellow_goal_list
        }

        data = {
            "goal_pos": goal_pos
        }

        with open(filename, 'a') as file:
            file.write(json.dumps(data, separators=(',', ':')))
            file.write('\n')

        print("Goal positions written to {} file.".format(filename.split('/')[-1]))
    except Exception as e:
        logging.error(f"An error occurred while writing goal positions to JSON file: {e}")


def main(WRITE="gpt3.5_dcpd1-1", IS_DOC=False, IS_VOICE=False):
    logging.info("Initializing TCP...")
    HOST = ''
    ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ss.bind((HOST, 5001))
    ss.listen(1)
    ss.setblocking(0)
    logging.info("Done.")

    logging.info("Initializing Simulator...")
    env = GraspingEnv()
    env.reset()
    logging.info("Done.")

    i=0
    while True:
        try:
            logging.info("Waiting for connection...")
            conn, addr = ss.accept()
            conn.setblocking(0)
            logging.info(f"Connected by {addr}")

            while True:
                try:
                    data = conn.recv(2048)
                    if not data:
                        break

                    code = extract_python_code(data.decode())

                    if code is not None:
                        i=i+1
                        if 'pri.say' in code:
                            IS_VOICE and text_to_speech(code[7:-1])
                        else:
                            # 定义停止事件
                            stop_event = threading.Event()
                            # 定义录制线程
                            video_filename = f"Iphone-video/window_video_{i}.avi"
                            record_thread = threading.Thread(target=record_window, args=("MuJoCo : empty_floor", video_filename, stop_event))
                            IS_VOICE and text_to_speech("收到，现在开始执行任务！")
                            # 开始录制视频
                            record_thread.start()
                            success = execute_python_code(env, code)  # 执行提取到的代码
                            # 停止录制视频
                            stop_event.set()
                            record_thread.join()
                            IS_VOICE and text_to_speech("任务执行完毕！")
                            if WRITE:
                                write_goal_positions_to_json(env,filename = f'output/{WRITE}.txt')  # 写入目标文件

                            if success:
                                if IS_DOC:
                                    env.step(env.action)
                                    env.reset()
                                else:
                                    env.step(env.action)
                    else:
                        logging.warning('\033[31m'":No code extracted, writing default goal positions.")
                        IS_VOICE and text_to_speech("这里没有提取到相应代码，请检查输入！")
                        if WRITE:
                            write_goal_positions_to_json(env,filename = f'output/{WRITE}.txt')  # 写入目标文件


                    conn.sendall("ACK".encode())  # 发送ACK确认消息

                except socket.error as e:
                    if e.errno == errno.EWOULDBLOCK:
                        pass
                    else:
                        raise
        except socket.error as e:
            if e.errno == errno.EWOULDBLOCK:
                pass
            else:
                raise


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Server for interacting with robot via various input methods.")
    parser.add_argument("--write", type=str, default="gpt3.5_dcpd1-1", help="Specify the write directory")
    parser.add_argument("--doc", action="store_true", help="Run in document input mode")
    parser.add_argument("--voice", action="store_true", help="Run in voice command mode")
    args = parser.parse_args()
    main(WRITE=args.write, IS_DOC=args.doc, IS_VOICE=args.voice)

