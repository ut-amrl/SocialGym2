from flask import Flask, render_template, request
import json
import os
import threading
from queue import Queue
from pathlib import Path
import sys
parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(1, str(parent_dir))
from config_runner.run import run 


app = Flask(__name__)

configs = Queue()

@app.route('/form')
def form():
    reports = get_eval_reports()
    return render_template("form.html", reports=reports)

@app.route('/upload', methods=['POST'])
def upload():
    # Access the uploaded file, and form fields
    checkpoint_file = request.files['checkpoint']
    policy_algo_sb3_contrib = request.form['policy_algo_sb3_contrib']
    policy_algo_name = request.form['policy_algo_name']
    policy_name = request.form['policy_name']
    n_steps = request.form['n_steps']
    
    # You can now save the file and use the form data to process further.
    # This example only returns a confirmation message.
    
    # Process and save the file, etc...
    # with open("log/template.json", 'r') as json_file:
    #     json_data = json.load(json_file)

    # os.makedirs(f"data", exist_ok=True)
    # checkpoint_file.save(f"data/{policy_name}.zip")

    # json_data['policy_algo_name'] = policy_algo_name
    # json_data['policy_name'] = policy_name
    # json_data['continue_from'] = f"data/{policy_name}.zip"
    # json_data['policy_algo_sb3_contrib'] = policy_algo_sb3_contrib
    # json_data['policy_algo_kwargs'] = {"n_steps": n_steps}

    # os.makedirs("config_runner/configs/log", exist_ok=True)
    # with open(f"config_runner/configs/log/{policy_name}.json", 'w') as updated_json_file:
    #     json.dump(json_data, updated_json_file, indent=2)
    # configs.put(f"log/{policy_name}.json")

    return render_template('popup.html',
                           policy_algo_sb3_contrib=policy_algo_sb3_contrib,
                           policy_algo_name=policy_algo_name,
                           policy_name=policy_name,
                           n_steps=n_steps)

def get_eval_reports():
    log_directory = os.path.join('..', 'log')
    reports = []

    for folder_name in os.listdir(log_directory):
        folder_path = os.path.join(log_directory, folder_name)
        if os.path.isdir(folder_path):
            report_path = os.path.join(folder_path, 'eval_report__SACADRL.json')
            if os.path.isfile(report_path):
                with open(report_path, 'r') as f:
                    report_data = json.load(f)
                    print(report_data.keys())
                    print(report_data)
                    print(report_data['3'])
                    report_data['3']['timestamp'] = folder_name
                    reports.append(report_data['3'])
    return reports

@app.route('/leaderboard')
def leaderboard():
    reports = get_eval_reports()
    return render_template('leaderboard.html', reports=reports)

def runner():
    print("Started Thread")
    while True:
        print("Waiting")
        config = configs.get(block=True)
        print("Got config: ", config)
        run([config])

if __name__ == "__main__":
    runner = threading.Thread(target=runner)
    runner.start() 
    app.run(debug=True, port=8080)
    runner.join()