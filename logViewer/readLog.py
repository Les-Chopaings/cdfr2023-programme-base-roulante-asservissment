import os
import re
import plotly.graph_objs as go
import base64
import subprocess
from datetime import datetime
import argparse

# === CONFIGURATION ===
ROBOT_HOST = "raspitronik.local"
ROBOT_USER = "robotronik"
REMOTE_DIR = "~/LOG_CDFR2025/"
LOCAL_CACHE_DIR = ".logCache"

# Crée le dossier local si nécessaire
os.makedirs(LOCAL_CACHE_DIR, exist_ok=True)

def run_command(command, **kwargs):
    print(f"\n[CMD] {' '.join(command)}")
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            **kwargs
        )
        if result.returncode != 0:
            print(f"[ERREUR] Code de retour : {result.returncode}")
            print(f"[STDERR] {result.stderr.strip()}")
            print(f"[STDOUT] {result.stdout.strip()}")
            raise subprocess.CalledProcessError(result.returncode, command)
        return result
    except subprocess.CalledProcessError as e:
        print(f"[EXCEPTION] Commande échouée : {e}")
        return None

# === Liste les fichiers locaux ===
def list_local_logs():
    asservFilePatern = re.compile(r'asserv_(-?\d+).log')
    finalList = []
    for f in os.listdir(LOCAL_CACHE_DIR):
        name = os.path.basename(f)
        file_match = asservFilePatern.search(name)
        if file_match:
            num = int(file_match.group(1))
            finalList.append((num, name))
    finalList.sort(key=lambda x: x[0])
    return finalList

# === Liste les fichiers distants via SSH ===
def list_remote_logs():
    asservFilePatern = re.compile(r'asserv_(-?\d+).log')
    finalList = []
    try:
        result = run_command(
            ["ssh", f"{ROBOT_USER}@{ROBOT_HOST}", f"ls {REMOTE_DIR}*.log"],
            timeout=5
        )
        if result.returncode != 0:
            raise Exception(result.stderr.strip())
        for f in result.stdout.strip().splitlines():
            name = os.path.basename(f)
            file_match = asservFilePatern.search(name)
            if file_match:
                num = int(file_match.group(1))
                finalList.append((num, name))
        finalList.sort(key=lambda x: x[0])
        return finalList
    except Exception as e:
        print(f"[INFO] Impossible de se connecter au robot ({ROBOT_HOST}).\nMotif : {e}")
        return []

# === Copier un fichier depuis le robot ===
def fetch_remote_file(filename):
    remote_path = f"{ROBOT_USER}@{ROBOT_HOST}:{REMOTE_DIR}{filename}"
    local_path = os.path.join(LOCAL_CACHE_DIR, filename)
    try:
        run_command(["scp", remote_path, local_path], check=True)
        return local_path
    except Exception as e:
        print(f"[ERREUR] Échec du transfert SCP : {e}")
        return []


# === INTERACTION UTILISATEUR ===

def selectLogFile():
    print("Fichiers distants disponibles sur le robot :")
    remote_files = list_remote_logs()
    for num, fname in remote_files:
        print(f"[R{num}] {fname}")

    print("\nFichiers locaux disponibles dans .logCache :")
    local_files = list_local_logs()
    for num, fname in local_files:
        print(f"[L{num}] {fname}")

    choice = input("\nChoisissez un fichier ([L<num>] ou [R<num>]) : ")
    return choice

# === Traitement du fichier choisi ===
coord_pattern = re.compile(r'(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}).*get_coordinates\s*:\s*x\s*(-?\d+),\s*y\s*(-?\d+),\s*theta\s*(-?\d+)')

event_patterns = [
    {
        "name": "go_to_point",
        "pattern": re.compile(r'(go_to_point.*)'),
        "color": "green",
        "symbol": "cross"
    },
    {
        "name": "stop",
        "pattern": re.compile(r'(stop.*)'),
        "color": "red",
        "symbol": "circle"
    },
    {
        "name": "pause",
        "pattern": re.compile(r'(pause.*)'),
        "color": "orange",
        "symbol": "diamond"
    },
    {
        "name": "resume",
        "pattern": re.compile(r'(resume.*)'),
        "color": "purple",
        "symbol": "triangle-up"
    },
    # Ajoute ici d'autres patterns si besoin
]

# Variables pour stocker les données

def annaliseLog(choice):
    selected_file = "asserv_" + choice[1:] + ".log"

    if choice.startswith("L"):
        log_file_path = os.path.join(LOCAL_CACHE_DIR, selected_file)
    elif choice.startswith("R"):
        log_file_path = fetch_remote_file(selected_file)
        print(f"Fichier '{selected_file}' copié depuis le robot.")
    else:
        raise ValueError("Choix invalide")

    x_coords = []
    y_coords = []
    theta_coords = []
    times = []

    event_points = {event["name"]: [] for event in event_patterns}

    last_position = None
    t0 = None

    with open(log_file_path, 'r') as f:
        for line in f:
            coord_match = coord_pattern.search(line)
            if coord_match:
                timestamp_str = coord_match.group(1)
                x = int(coord_match.group(2))
                y = int(coord_match.group(3))
                theta = int(coord_match.group(3))
                ts = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                if t0 is None:
                    t0 = ts
                    t_sec = 0.0
                else:
                    t_sec = (ts - t0).total_seconds()
                last_position = (x, y)
                x_coords.append(x)
                y_coords.append(y)
                theta_coords.append(theta)
                times.append(t_sec)
                continue

            for event in event_patterns:
                match = event["pattern"].search(line)
                if match and last_position and t0:
                    label = match.group(1).strip()
                    event_points[event["name"]].append(
                        (last_position[0], last_position[1], label)
                    )
                    break

    # === Traces Plotly ===
    # Trace principale du chemin
    trace_path = go.Scatter(
        y=x_coords,
        x=y_coords,
        mode='lines',
        name='Trajet',
        line=dict(color='blue')
    )

    # Traces dynamiques pour les événements
    event_traces = []
    for event in event_patterns:
        points = event_points[event["name"]]
        if not points:
            continue
        trace = go.Scatter(
            y=[p[0] for p in points],
            x=[p[1] for p in points],
            mode='markers',
            name=event["name"],
            marker=dict(color=event["color"], size=10, symbol=event["symbol"]),
            text=[p[2] for p in points],
            hoverinfo='text'
        )
        event_traces.append(trace)
    # =========================
    # TRACE DU ROBOT (ANIMÉE)
    # =========================
    robot_trace = go.Scatter(
        x=[y_coords[0]],
        y=[x_coords[0]],
        mode='markers',
        marker=dict(color='red', size=12),
        name='Robot'
    )

    # =========================
    # FRAMES (uniquement si le temps change)
    # =========================
    frames = []
    last_time = None
    robot_trace_index = len([trace_path] + event_traces)  # index exact du robot

    for i in range(len(x_coords)):
        if times[i] != last_time:
            frames.append(
                go.Frame(
                    data=[dict(x=[y_coords[i]], y=[x_coords[i]])],
                    traces=[robot_trace_index],
                    name=str(i)
                )
            )
            last_time = times[i]

    # =========================
    # SLIDER (barre d'avancement)
    # =========================
    slider = [dict(
        active=0,
        steps=[
            dict(
                label=f"{times[int(frame.name)]:.1f}s",
                method="animate",
                args=[
                    [frame.name],
                    {"mode": "immediate",
                    "frame": {"duration": 0, "redraw": False},
                    "transition": {"duration": 0}}
                ]
            )
            for frame in frames
        ],
        x=0.1,
        y=0,
        len=0.8
    )]

    # =========================
    # IMAGE DE FOND (SVG)
    # =========================
    image_path = "../../../ressource/table.svg"
    with open(image_path, "r", encoding="utf-8") as svg_file:
        svg_content = svg_file.read()
    image_base64 = "data:image/svg+xml;base64," + base64.b64encode(
        svg_content.encode("utf-8")
    ).decode()

    # =========================
    # LAYOUT + CONTROLES
    # =========================
    layout = go.Layout(
        title='Animation du robot',
        xaxis=dict(title='Y', range=[-1500, 1500], scaleanchor='y'),
        yaxis=dict(title='X', range=[1000, -1000]),
        showlegend=True,
        images=[dict(
            source=image_base64,
            xref="x",
            yref="y",
            x=-1500,
            y=-1000,
            sizex=3000,
            sizey=2000,
            sizing="stretch",
            opacity=1,
            layer="below"
        )],
        sliders=slider,
        updatemenus=[dict(
            type="buttons",
            showactive=True,
            x=1.05,
            y=0.5,
            buttons=[
                dict(
                    label="▶ x1",
                    method="animate",
                    args=[None, {
                        "frame": {"duration": 1000, "redraw": True},
                        "fromcurrent": True,
                        "transition": {"duration": 0}
                    }]
                ),
                dict(
                    label="▶ xX",
                    method="animate",
                    args=[None, {
                        "frame": {"duration": 50, "redraw": True},
                        "fromcurrent": True,
                        "transition": {"duration": 0}
                    }]
                ),
                dict(
                    label="⏸ Pause",
                    method="animate",
                    args=[[None], {
                        "mode": "immediate",
                        "frame": {"duration": 0, "redraw": False},
                        "transition": {"duration": 0}
                    }]
                )
            ]
        )]
    )

    # =========================
    # FIGURE FINALE
    # =========================
    fig = go.Figure(
        data=[trace_path] + event_traces + [robot_trace],
        layout=layout,
        frames=frames
    )

    fig.show()


def main():
    parser = argparse.ArgumentParser(description="Lecture de logs")

    parser.add_argument(
        "log_id",
        nargs="?",  # rend l'argument optionnel
        help="ID du log à lire (ex: L1295)"
    )
    args = parser.parse_args()

    if args.log_id:
        log_file_path = f"{args.log_id}"
        annaliseLog(log_file_path)
    else:
        annaliseLog(selectLogFile())

if __name__ == "__main__":
    main()