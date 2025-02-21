import rosbag
import tf
import numpy as np
import pandas as pd
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
import matplotlib
#matplotlib.use('Agg')  # Nutzt ein Backend ohne GUI (kein Qt)



# Transformation vom Messsystem ins Karten-Koordinatensystem
translation_offset = np.array([39.2691+0.1710, 31.8942, 0])
rotation_angle = 3.1656  # Rotation um die Z-Achse

def rotate_z(points, angle):
    """ Rotiert Punkte um die Z-Achse. """
    R = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,              0,             1]
    ])
    return points @ R.T

def extract_tf_data(bag_file):
    """ Extrahiert die Positionen von 'virtual_object/base_link' und 'leiter' aus einer ROS-Bag. """
    virtual_object_positions = []
    leiter_positions = []
    
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                
                # Position extrahieren
                position = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                
                # Daten sammeln
                if frame_id == "virtual_object/base_link":
                    virtual_object_positions.append(position)
                elif frame_id == "leiter":
                    # Transformation ins Karten-Koordinatensystem
                    transformed_position = rotate_z(position - translation_offset, rotation_angle)
                    leiter_positions.append(transformed_position)
    
    # Umwandlung in DataFrames
    virtual_object_df = pd.DataFrame(virtual_object_positions, columns=["x", "y", "z"])
    leiter_df = pd.DataFrame(leiter_positions, columns=["x", "y", "z"])
    
    return virtual_object_df, leiter_df


def synchronize_data(virtual_object_df, leiter_df):
    """ Interpoliert die Daten von 'leiter', damit beide Datensätze die gleiche Anzahl an Zeitpunkten haben. """
    n_points = min(len(virtual_object_df), len(leiter_df))

    # Interpolation für x, y, z in Leiter-Daten
    interp_x = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["x"], kind="linear", fill_value="extrapolate")
    interp_y = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["y"], kind="linear", fill_value="extrapolate")
    interp_z = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["z"], kind="linear", fill_value="extrapolate")

    leiter_df_interp = pd.DataFrame({
        "x": interp_x(np.linspace(0, 1, n_points)),
        "y": interp_y(np.linspace(0, 1, n_points)),
        "z": interp_z(np.linspace(0, 1, n_points))
    })

    # Gleiche Anzahl an Punkten sicherstellen
    virtual_object_df = virtual_object_df.iloc[:n_points].reset_index(drop=True)
    leiter_df_interp = leiter_df_interp.reset_index(drop=True)

    return virtual_object_df, leiter_df_interp





def compute_error(virtual_object_df, leiter_df):
    """ Berechnet den Fehler zwischen der transformierten Leiter-Trajektorie und dem virtuellen Objekt. """
    if len(virtual_object_df) != len(leiter_df):
        print("⚠️ Achtung: Unterschiedliche Anzahl an Zeitpunkten! Die Daten sollten synchronisiert werden.")
        return None

    errors = np.linalg.norm(virtual_object_df.to_numpy() - leiter_df.to_numpy(), axis=1)
    
    error_stats = {
        "Mean Error": np.mean(errors),
        "Max Error": np.max(errors),
        "RMSE": np.sqrt(np.mean(errors**2)),
    }

    # Fehler pro Achse
    error_stats.update({
        "Mean Error (x)": np.mean(np.abs(virtual_object_df["x"] - leiter_df["x"])),
        "Max Error (x)": np.max(np.abs(virtual_object_df["x"] - leiter_df["x"])),
        "RMSE (x)": np.sqrt(np.mean((virtual_object_df["x"] - leiter_df["x"])**2)),

        "Mean Error (y)": np.mean(np.abs(virtual_object_df["y"] - leiter_df["y"])),
        "Max Error (y)": np.max(np.abs(virtual_object_df["y"] - leiter_df["y"])),
        "RMSE (y)": np.sqrt(np.mean((virtual_object_df["y"] - leiter_df["y"])**2)),

        "Mean Error (z)": np.mean(np.abs(virtual_object_df["z"] - leiter_df["z"])),
        "Max Error (z)": np.max(np.abs(virtual_object_df["z"] - leiter_df["z"])),
        "RMSE (z)": np.sqrt(np.mean((virtual_object_df["z"] - leiter_df["z"])**2)),
    })

    return error_stats

def plot_trajectories(virtual_object_df, leiter_df):
    """ Erstellt eine 3D-Visualisierung der Trajektorien. """
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Originale Soll-Daten (Virtual Object) plotten
    ax.plot(virtual_object_df["x"], virtual_object_df["y"], virtual_object_df["z"], label="Virtual Object", color="blue", alpha=0.6)

    # Transformierte Ist-Daten (Leiter) plotten
    ax.plot(leiter_df["x"], leiter_df["y"], leiter_df["z"], label="Leiter (transformed)", color="red", alpha=0.6)

    ax.set_xlabel("X-Koordinate")
    ax.set_ylabel("Y-Koordinate")
    ax.set_zlabel("Z-Koordinate")
    ax.set_title("Vergleich der Trajektorien nach Transformation")
    ax.legend()

    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Analyse der Leiter- und Virtual Object-Trajektorien aus einer ROS-Bag.")
    parser.add_argument("bag_file", type=str, help="Pfad zur ROS-Bag-Datei")

    args = parser.parse_args()
    virtual_object_df, leiter_df = extract_tf_data(args.bag_file)

    # Rufe die Funktion nach der Extraktion auf:
    virtual_object_df, leiter_df = synchronize_data(virtual_object_df, leiter_df)
    
    if virtual_object_df.empty or leiter_df.empty:
        print("❌ Keine relevanten /tf-Daten gefunden.")
        return
    
    errors = compute_error(virtual_object_df, leiter_df)
    
    if errors:
        print("\n🔍 Fehleranalyse:")
        for key, value in errors.items():
            print(f"{key}: {value:.4f} m")

    plot_trajectories(virtual_object_df, leiter_df)

if __name__ == "__main__":
    main()
