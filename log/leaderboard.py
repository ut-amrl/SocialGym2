from flask import Flask, render_template
import json
import os

app = Flask(__name__)

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

@app.route('/')
def leaderboard():
    reports = get_eval_reports()
    return render_template('leaderboard.html', reports=reports)

if __name__ == "__main__":
    app.run(debug=True, port=8000)