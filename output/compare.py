import numpy as np
import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--file", required=True, type=str)
args = parser.parse_args()


def load_txt_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    return [json.loads(line) for line in lines]

def load_json_data(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return [item['goal_pos'] for item in data]

def calculate_error(pos1, pos2):
    pos1_arr = np.array(pos1)
    pos2_arr = np.array(pos2)
    return np.sum(np.abs(pos1_arr - pos2_arr))

def compare_positions(txt_data, json_data, threshold=0.05):
    correct_count = 0
    sum_error = 0
    total_count = len(txt_data)
    incorrect_lines = []
    detailed_errors = []
    
    for i, (txt_item, json_item) in enumerate(zip(txt_data, json_data)):
        json_item = json.loads(json_item)
        gripper_pos_error = calculate_error(txt_item['goal_pos']['gripper_pos'],json_item['gripper_pos'])
        red_goal_error = calculate_error(txt_item['goal_pos']['red_goal'], json_item['red_goal'])
        blue_goal_error = calculate_error(txt_item['goal_pos']['blue_goal'], json_item['blue_goal'])
        green_goal_error = calculate_error(txt_item['goal_pos']['green_goal'], json_item['green_goal'])
        yellow_goal_error = calculate_error(txt_item['goal_pos']['yellow_goal'], json_item['yellow_goal'])
        if txt_item['goal_pos']['gripper_state'] == json_item['gripper_state']:
            gripper_state_error = 0
        else:
            gripper_state_error = 0.05

        total_entry_error = gripper_pos_error + red_goal_error + blue_goal_error + green_goal_error + yellow_goal_error + gripper_state_error
        sum_error += total_entry_error

        if total_entry_error < threshold:
            correct_count += 1
        else:
            incorrect_lines.append(i + 1)  # Store the line number (1-based)
            detailed_errors.append({
                'line': i + 1,
                'gripper_pos_error': gripper_pos_error,
                'red_goal_error': red_goal_error,
                'blue_goal_error': blue_goal_error,
                'green_goal_error': green_goal_error,
                'yellow_goal_error': yellow_goal_error,
                'gripper_state_error': gripper_state_error,
                'total_error': total_entry_error
            })
    
    accuracy = correct_count / total_count
    mean_error = sum_error / total_count
    return accuracy, incorrect_lines, detailed_errors, mean_error


json_file_path = '../dataset/dcpd1-1.json'

txt_data = load_txt_data(args.file)
json_data = load_json_data(json_file_path)
accuracy, incorrect_lines, detailed_errors, mean_error = compare_positions(txt_data, json_data)

print(f"Accuracy: {accuracy * 100:.2f}%")
print(f"Mean error: {mean_error}")
if incorrect_lines:
    print(f"Incorrect lines: {incorrect_lines}")
    for error_detail in detailed_errors:
        # print(f"Line {error_detail['line']}:")
        # print(f"  Gripper Position Error: {error_detail['gripper_pos_error']}")
        # print(f"  Gripper State Error: {error_detail['gripper_state_error']}")
        # print(f"  Red Goal Error: {error_detail['red_goal_error']}")
        # print(f"  Blue Goal Error: {error_detail['blue_goal_error']}")
        # print(f"  Green Goal Error: {error_detail['green_goal_error']}")
        # print(f"  Yellow Goal Error: {error_detail['yellow_goal_error']}")
        # print(f"  Total Error: {error_detail['total_error']}")

        print(f"{error_detail['line']}:{error_detail['total_error']}")
else:
    print("All lines are correct.")

