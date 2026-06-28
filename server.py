from flask import Flask, request, jsonify
from flask_cors import CORS
import os

app = Flask(__name__)
CORS(app)

latest_cmd = {
    "seq": 0,
    "dir": ""
}

@app.route("/")
def home():
    return "ShadeBike command server is running."

@app.route("/set")
def set_command():
    direction = request.args.get("dir", "")

    if direction not in ["L", "R", "S", "A", "X"]:
        return jsonify({
            "ok": False,
            "message": "Invalid direction"
        }), 400

    latest_cmd["seq"] += 1
    latest_cmd["dir"] = direction

    print(f"New command: seq={latest_cmd['seq']}, dir={latest_cmd['dir']}")

    return jsonify({
        "ok": True,
        "seq": latest_cmd["seq"],
        "dir": latest_cmd["dir"]
    })

@app.route("/get")
def get_command():
    return jsonify(latest_cmd)

@app.route("/get_plain")
def get_plain():
    return f"{latest_cmd['seq']},{latest_cmd['dir']}"

if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5050))
    app.run(host="0.0.0.0", port=port)
